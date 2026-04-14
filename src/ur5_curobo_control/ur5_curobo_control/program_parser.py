#!/usr/bin/env python3
"""
Robot Program Parser for UR5 Control

Parses program files with robot instructions like:
  - movetopose([x, y, z], [qw, qx, qy, qz])
  - movetojoint([j1, j2, j3, j4, j5, j6])  # Joint angles in DEGREES
  - movetonamed(PositionName)                # Named position from config
  - moverelative(direction, distance)         # Relative Cartesian move
  - runprogram(filename.prog)                 # Execute another .prog file
  - wait(seconds)
  - opengripper
  - closegripper
  - gripper(position)  # 0.0 = open, 1.0 = closed
  - set_speed(factor)  # 0.0-1.0 velocity scaling
  - force_mode([x,y,z,rx,ry,rz], [fx,fy,fz,tx,ty,tz], speed_limit)
  - force_mode_stop()
  - movel(direction, distance, speed, accel)  # URScript linear move (works during force mode)
  - movel([x,y,z], distance, speed, accel)   # movel with arbitrary direction vector
  - # comments are ignored

  Conditional blocks:
    if gripper_open / if gripper_closed
    if near(PositionName) / if near(PositionName, tolerance)
    if not_near(PositionName) / if not_near(PositionName, tolerance)
    else
    endif

Note: Joint angles in program files and named_positions.txt are specified
      in DEGREES and automatically converted to radians by the parser.
"""

import re
from dataclasses import dataclass
from typing import List, Tuple, Optional, Union
from enum import Enum, auto


class InstructionType(Enum):
    MOVE_TO_POSE = auto()
    MOVE_TO_JOINT = auto()
    MOVE_TO_NAMED = auto()
    MOVE_RELATIVE = auto()
    RUN_PROGRAM = auto()
    IF = auto()
    ELSE = auto()
    ENDIF = auto()
    WAIT = auto()
    OPEN_GRIPPER = auto()
    CLOSE_GRIPPER = auto()
    GRIPPER = auto()
    SET_SPEED = auto()
    FORCE_MODE = auto()
    FORCE_MODE_STOP = auto()
    MOVEL = auto()
    JOINT_DELTA = auto()
    SET_STEP_TIME = auto()
    COMMENT = auto()
    UNKNOWN = auto()


@dataclass
class RobotInstruction:
    """Represents a single robot instruction."""
    type: InstructionType
    line_number: int
    raw_line: str
    # Pose: ([x,y,z], [qw,qx,qy,qz])
    pose: Optional[Tuple[List[float], List[float]]] = None
    # Joint positions: [j1,j2,j3,j4,j5,j6]
    joint_positions: Optional[List[float]] = None
    # Wait duration in seconds
    wait_duration: Optional[float] = None
    # Gripper position (0.0 = open, 1.0 = closed)
    gripper_position: Optional[float] = None
    # Speed factor (0.0 - 1.0)
    speed_factor: Optional[float] = None
    # Comment text
    comment: Optional[str] = None
    # Named position name (for movetonamed)
    named_position: Optional[str] = None
    # Relative move: (direction_or_vector, distance_meters, reference_frame)
    relative_move: Optional[Tuple[Union[str, List[float]], float, str]] = None
    # Sub-program filename (for runprogram)
    sub_program: Optional[str] = None
    # Condition string (for IF instructions, evaluated at runtime)
    condition: Optional[str] = None
    # Condition parameters (parsed from the condition string)
    condition_type: Optional[str] = None  # 'gripper_open', 'gripper_closed', 'near', 'not_near'
    condition_target: Optional[str] = None  # position name for near/not_near
    condition_tolerance: Optional[float] = None  # tolerance in radians for near/not_near
    # Force mode parameters
    force_mode_axes: Optional[List[bool]] = None  # [x,y,z,rx,ry,rz] True=compliant
    force_mode_wrench: Optional[List[float]] = None  # [fx,fy,fz,tx,ty,tz] in N/Nm
    force_mode_speed_limit: Optional[float] = None  # m/s for compliant axes
    # movel parameters
    movel_direction: Optional[List[float]] = None  # [x,y,z] direction vector (normalized)
    movel_distance: Optional[float] = None  # meters
    movel_speed: Optional[float] = None  # m/s linear speed
    movel_accel: Optional[float] = None  # m/s^2 acceleration
    # Joint delta parameters (for JOINT_DELTA instruction)
    joint_delta: Optional[List[float]] = None  # [d1,d2,d3,d4,d5,d6] in radians
    # Step time for SET_STEP_TIME instruction
    step_time: Optional[float] = None  # seconds per delta step


class ProgramParser:
    """Parses robot program files into a list of instructions."""
    
    # Regex patterns for parsing instructions
    MOVE_TO_POSE_PATTERN = re.compile(
        r'movetopose\s*\(\s*'
        r'\[([^\]]+)\]\s*,\s*'  # Position [x, y, z]
        r'\[([^\]]+)\]\s*\)',    # Quaternion [qw, qx, qy, qz]
        re.IGNORECASE
    )
    
    MOVE_TO_JOINT_PATTERN = re.compile(
        r'movetojoint\s*\(\s*\[([^\]]+)\]\s*\)',
        re.IGNORECASE
    )
    
    WAIT_PATTERN = re.compile(
        r'wait\s*\(\s*([0-9.]+)\s*\)',
        re.IGNORECASE
    )
    
    GRIPPER_PATTERN = re.compile(
        r'gripper\s*\(\s*([0-9.]+)\s*\)',
        re.IGNORECASE
    )
    
    SET_SPEED_PATTERN = re.compile(
        r'set_speed\s*\(\s*([0-9.]+)\s*\)',
        re.IGNORECASE
    )
    
    OPEN_GRIPPER_PATTERN = re.compile(r'opengripper', re.IGNORECASE)
    CLOSE_GRIPPER_PATTERN = re.compile(r'closegripper', re.IGNORECASE)
    COMMENT_PATTERN = re.compile(r'^\s*#(.*)$')
    
    # New patterns for extended instructions
    MOVE_TO_NAMED_PATTERN = re.compile(
        r'movetonamed\s*\(\s*([\w]+)\s*\)',
        re.IGNORECASE
    )
    
    MOVE_RELATIVE_NAMED_DIR_PATTERN = re.compile(
        r'moverelative\s*\(\s*(?:([a-zA-Z_]\w*|[\"\'][a-zA-Z_]\w*[\"\'])\s*,\s*)?([a-zA-Z_]\w*)\s*(?:,\s*([0-9.]+))?\s*\)',
        re.IGNORECASE
    )

    MOVE_RELATIVE_VECTOR_PATTERN = re.compile(
        r'moverelative\s*\(\s*(?:([a-zA-Z_]\w*|[\"\'][a-zA-Z_]\w*[\"\'])\s*,\s*)?\[([^\]]+)\]\s*(?:,\s*([0-9.]+))?\s*\)',
        re.IGNORECASE
    )
    
    RUN_PROGRAM_PATTERN = re.compile(
        r'runprogram\s*\(\s*([\w./-]+)\s*\)',
        re.IGNORECASE
    )

    # Force mode patterns
    # force_mode(axes, wrench)  or  force_mode(axes, wrench, speed_limit)
    # axes: [0,0,1,0,0,0]  wrench: [0,0,-10,0,0,0]  speed_limit: 0.05
    FORCE_MODE_PATTERN = re.compile(
        r'force_mode\s*\(\s*'
        r'\[([^\]]+)\]\s*,\s*'        # axes [0,0,1,0,0,0]
        r'\[([^\]]+)\]\s*'            # wrench [0,0,-10,0,0,0]
        r'(?:,\s*([0-9.]+)\s*)?'      # optional speed_limit
        r'\)',
        re.IGNORECASE
    )
    FORCE_MODE_STOP_PATTERN = re.compile(r'force_mode_stop\s*\(\s*\)', re.IGNORECASE)

    # movel with named direction: movel(forward, 0.15, 0.05)
    # movel with vector: movel([0,1,-1], 0.15, 0.05)
    # Uses URScript movel via urscript_interface - works during force mode
    MOVEL_NAMED_PATTERN = re.compile(
        r'movel\s*\(\s*(\w+)\s*,\s*([0-9.]+)'
        r'(?:\s*,\s*([0-9.]+))?'           # optional speed
        r'(?:\s*,\s*([0-9.]+))?'           # optional accel
        r'\s*\)',
        re.IGNORECASE
    )
    MOVEL_VECTOR_PATTERN = re.compile(
        r'movel\s*\(\s*\[([^\]]+)\]\s*,\s*([0-9.]+)'
        r'(?:\s*,\s*([0-9.]+))?'           # optional speed
        r'(?:\s*,\s*([0-9.]+))?'           # optional accel
        r'\s*\)',
        re.IGNORECASE
    )

    # jointdelta([d1,d2,d3,d4,d5,d6])  — apply incremental joint-space delta (radians)
    JOINT_DELTA_PATTERN = re.compile(
        r'jointdelta\s*\(\s*\[([^\]]+)\]\s*\)',
        re.IGNORECASE
    )

    # set_step_time(seconds)  — set duration for subsequent jointdelta steps
    SET_STEP_TIME_PATTERN = re.compile(
        r'set_step_time\s*\(\s*([0-9.]+)\s*\)',
        re.IGNORECASE
    )

    # Conditional patterns
    # if gripper_open / if gripper_closed
    IF_GRIPPER_PATTERN = re.compile(
        r'^if\s+(gripper_open|gripper_closed)\s*$',
        re.IGNORECASE
    )
    # if near(PositionName) / if near(PositionName, 0.1)
    IF_NEAR_PATTERN = re.compile(
        r'^if\s+(near|not_near)\s*\(\s*([\w]+)\s*(?:,\s*([0-9.]+))?\s*\)\s*$',
        re.IGNORECASE
    )
    ELSE_PATTERN = re.compile(r'^else\s*$', re.IGNORECASE)
    ENDIF_PATTERN = re.compile(r'^endif\s*$', re.IGNORECASE)
    
    def __init__(self):
        self.instructions: List[RobotInstruction] = []
        self.errors: List[str] = []
    
    def parse_file(self, filepath: str) -> List[RobotInstruction]:
        """Parse a program file and return list of instructions."""
        self.instructions = []
        self.errors = []
        
        with open(filepath, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        for line_num, line in enumerate(lines, start=1):
            instruction = self._parse_line(line.strip(), line_num)
            if instruction and instruction.type != InstructionType.COMMENT:
                self.instructions.append(instruction)
        
        return self.instructions
    
    def parse_string(self, program_text: str) -> List[RobotInstruction]:
        """Parse a program string and return list of instructions."""
        self.instructions = []
        self.errors = []
        
        lines = program_text.strip().split('\n')
        
        for line_num, line in enumerate(lines, start=1):
            instruction = self._parse_line(line.strip(), line_num)
            if instruction and instruction.type != InstructionType.COMMENT:
                self.instructions.append(instruction)
        
        return self.instructions
    
    def _parse_line(self, line: str, line_num: int) -> Optional[RobotInstruction]:
        """Parse a single line into a RobotInstruction."""
        # Skip empty lines
        if not line:
            return None
        
        # Check for comment
        comment_match = self.COMMENT_PATTERN.match(line)
        if comment_match:
            return RobotInstruction(
                type=InstructionType.COMMENT,
                line_number=line_num,
                raw_line=line,
                comment=comment_match.group(1).strip()
            )
        
        # Strip inline comments
        if '#' in line:
            line = line[:line.index('#')].strip()
        
        if not line:
            return None
        
        # Check for movetopose
        match = self.MOVE_TO_POSE_PATTERN.search(line)
        if match:
            try:
                position = [float(x.strip()) for x in match.group(1).split(',')]
                quaternion = [float(x.strip()) for x in match.group(2).split(',')]
                if len(position) != 3 or len(quaternion) != 4:
                    self.errors.append(f"Line {line_num}: Invalid pose dimensions")
                    return None
                return RobotInstruction(
                    type=InstructionType.MOVE_TO_POSE,
                    line_number=line_num,
                    raw_line=line,
                    pose=(position, quaternion)
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse pose: {e}")
                return None
        
        # Check for movetojoint
        match = self.MOVE_TO_JOINT_PATTERN.search(line)
        if match:
            try:
                joints = [float(x.strip()) for x in match.group(1).split(',')]
                if len(joints) != 6:
                    self.errors.append(f"Line {line_num}: Expected 6 joint values, got {len(joints)}")
                    return None
                # Convert from degrees to radians
                joints_rad = [j * 3.14159265359 / 180.0 for j in joints]
                return RobotInstruction(
                    type=InstructionType.MOVE_TO_JOINT,
                    line_number=line_num,
                    raw_line=line,
                    joint_positions=joints_rad
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse joint values: {e}")
                return None
        
        # Check for wait
        match = self.WAIT_PATTERN.search(line)
        if match:
            try:
                duration = float(match.group(1))
                return RobotInstruction(
                    type=InstructionType.WAIT,
                    line_number=line_num,
                    raw_line=line,
                    wait_duration=duration
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse wait duration: {e}")
                return None
        
        # Check for gripper(position)
        match = self.GRIPPER_PATTERN.search(line)
        if match:
            try:
                position = float(match.group(1))
                return RobotInstruction(
                    type=InstructionType.GRIPPER,
                    line_number=line_num,
                    raw_line=line,
                    gripper_position=position
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse gripper position: {e}")
                return None
        
        # Check for set_speed
        match = self.SET_SPEED_PATTERN.search(line)
        if match:
            try:
                speed = float(match.group(1))
                return RobotInstruction(
                    type=InstructionType.SET_SPEED,
                    line_number=line_num,
                    raw_line=line,
                    speed_factor=speed
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse speed factor: {e}")
                return None
        
        # Check for opengripper
        if self.OPEN_GRIPPER_PATTERN.search(line):
            return RobotInstruction(
                type=InstructionType.OPEN_GRIPPER,
                line_number=line_num,
                raw_line=line,
                gripper_position=0.0
            )
        
        # Check for closegripper
        if self.CLOSE_GRIPPER_PATTERN.search(line):
            return RobotInstruction(
                type=InstructionType.CLOSE_GRIPPER,
                line_number=line_num,
                raw_line=line,
                gripper_position=1.0
            )
        
        # Check for movetonamed(PositionName)
        match = self.MOVE_TO_NAMED_PATTERN.search(line)
        if match:
            name = match.group(1)
            return RobotInstruction(
                type=InstructionType.MOVE_TO_NAMED,
                line_number=line_num,
                raw_line=line,
                named_position=name
            )
        
        # Check for moverelative(direction, distance) or moverelative(ref_pos, direction, distance)
        match = self.MOVE_RELATIVE_NAMED_DIR_PATTERN.search(line)
        if match:
            ref_pos = match.group(1).strip(' "\'') if match.group(1) else "current"
            direction = match.group(2).lower()
            distance = float(match.group(3)) if match.group(3) else 0.05  # default 5cm
            valid_directions = ['left', 'right', 'forward', 'back', 'up', 'down']
            if direction not in valid_directions:
                self.errors.append(f"Line {line_num}: Invalid direction '{direction}'. Use: {', '.join(valid_directions)}")
                return None
            return RobotInstruction(
                type=InstructionType.MOVE_RELATIVE,
                line_number=line_num,
                raw_line=line,
                relative_move=(direction, distance, ref_pos)
            )

        # Check for moverelative([x,y,z], distance) or moverelative(ref_pos, [x,y,z], distance)
        match = self.MOVE_RELATIVE_VECTOR_PATTERN.search(line)
        if match:
            ref_pos = match.group(1).strip(' "\'') if match.group(1) else "current"
            try:
                vec = [float(x.strip()) for x in match.group(2).split(',')]
                if len(vec) != 3:
                    self.errors.append(f"Line {line_num}: moverelative vector must have 3 values [x,y,z]")
                    return None
                distance = float(match.group(3)) if match.group(3) else 0.05
                # Normalize the vector
                import math
                mag = math.sqrt(sum(v*v for v in vec))
                if mag < 1e-10:
                    self.errors.append(f"Line {line_num}: moverelative vector cannot be zero")
                    return None
                direction = [v / mag for v in vec]
                return RobotInstruction(
                    type=InstructionType.MOVE_RELATIVE,
                    line_number=line_num,
                    raw_line=line,
                    relative_move=(direction, distance, ref_pos)
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse moverelative vector: {e}")
                return None
        
        # Check for movel([x,y,z], distance, speed, accel) - vector form
        match = self.MOVEL_VECTOR_PATTERN.search(line)
        if match:
            try:
                vec = [float(x.strip()) for x in match.group(1).split(',')]
                if len(vec) != 3:
                    self.errors.append(f"Line {line_num}: movel vector must have 3 values [x,y,z]")
                    return None
                distance = float(match.group(2))
                speed = float(match.group(3)) if match.group(3) else None
                accel = float(match.group(4)) if match.group(4) else None
                # Normalize the direction vector
                import math
                mag = math.sqrt(sum(v*v for v in vec))
                if mag < 1e-10:
                    self.errors.append(f"Line {line_num}: movel direction vector cannot be zero")
                    return None
                direction = [v / mag for v in vec]
                return RobotInstruction(
                    type=InstructionType.MOVEL,
                    line_number=line_num,
                    raw_line=line,
                    movel_direction=direction,
                    movel_distance=distance,
                    movel_speed=speed,
                    movel_accel=accel
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse movel: {e}")
                return None

        # Check for movel(direction_name, distance, speed, accel) - named form
        match = self.MOVEL_NAMED_PATTERN.search(line)
        if match:
            direction_name = match.group(1).lower()
            distance = float(match.group(2))
            speed = float(match.group(3)) if match.group(3) else None
            accel = float(match.group(4)) if match.group(4) else None
            direction_map = {
                'left':    [0.0, +1.0, 0.0],
                'right':   [0.0, -1.0, 0.0],
                'forward': [+1.0, 0.0, 0.0],
                'back':    [-1.0, 0.0, 0.0],
                'up':      [0.0, 0.0, +1.0],
                'down':    [0.0, 0.0, -1.0],
            }
            if direction_name not in direction_map:
                self.errors.append(f"Line {line_num}: Invalid movel direction '{direction_name}'. Use: {', '.join(direction_map.keys())} or [x,y,z] vector")
                return None
            return RobotInstruction(
                type=InstructionType.MOVEL,
                line_number=line_num,
                raw_line=line,
                movel_direction=direction_map[direction_name],
                movel_distance=distance,
                movel_speed=speed,
                movel_accel=accel
            )

        # Check for jointdelta([d1,...,d6])
        match = self.JOINT_DELTA_PATTERN.search(line)
        if match:
            try:
                deltas = [float(x.strip()) for x in match.group(1).split(',')]
                if len(deltas) != 6:
                    self.errors.append(f"Line {line_num}: Expected 6 delta values, got {len(deltas)}")
                    return None
                return RobotInstruction(
                    type=InstructionType.JOINT_DELTA,
                    line_number=line_num,
                    raw_line=line,
                    joint_delta=deltas
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse jointdelta: {e}")
                return None

        # Check for set_step_time(seconds)
        match = self.SET_STEP_TIME_PATTERN.search(line)
        if match:
            try:
                t = float(match.group(1))
                return RobotInstruction(
                    type=InstructionType.SET_STEP_TIME,
                    line_number=line_num,
                    raw_line=line,
                    step_time=t
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse set_step_time: {e}")
                return None

        # Check for runprogram(filename)
        match = self.RUN_PROGRAM_PATTERN.search(line)
        if match:
            filename = match.group(1)
            return RobotInstruction(
                type=InstructionType.RUN_PROGRAM,
                line_number=line_num,
                raw_line=line,
                sub_program=filename
            )
        
        # Check for force_mode_stop() - must check before force_mode()
        if self.FORCE_MODE_STOP_PATTERN.search(line):
            return RobotInstruction(
                type=InstructionType.FORCE_MODE_STOP,
                line_number=line_num,
                raw_line=line
            )

        # Check for force_mode([axes], [wrench], speed_limit)
        match = self.FORCE_MODE_PATTERN.search(line)
        if match:
            try:
                axes = [int(float(x.strip())) != 0 for x in match.group(1).split(',')]
                wrench = [float(x.strip()) for x in match.group(2).split(',')]
                speed_limit = float(match.group(3)) if match.group(3) else 0.05
                if len(axes) != 6 or len(wrench) != 6:
                    self.errors.append(f"Line {line_num}: force_mode axes and wrench must have 6 values")
                    return None
                return RobotInstruction(
                    type=InstructionType.FORCE_MODE,
                    line_number=line_num,
                    raw_line=line,
                    force_mode_axes=axes,
                    force_mode_wrench=wrench,
                    force_mode_speed_limit=speed_limit
                )
            except ValueError as e:
                self.errors.append(f"Line {line_num}: Failed to parse force_mode: {e}")
                return None

        # Check for conditional: if gripper_open / if gripper_closed
        match = self.IF_GRIPPER_PATTERN.match(line)
        if match:
            cond_type = match.group(1).lower()
            return RobotInstruction(
                type=InstructionType.IF,
                line_number=line_num,
                raw_line=line,
                condition=line,
                condition_type=cond_type
            )
        
        # Check for conditional: if near(Name) / if not_near(Name, tolerance)
        match = self.IF_NEAR_PATTERN.match(line)
        if match:
            cond_type = match.group(1).lower()  # 'near' or 'not_near'
            target_name = match.group(2)
            tolerance = float(match.group(3)) if match.group(3) else None
            return RobotInstruction(
                type=InstructionType.IF,
                line_number=line_num,
                raw_line=line,
                condition=line,
                condition_type=cond_type,
                condition_target=target_name,
                condition_tolerance=tolerance
            )
        
        # Check for else
        if self.ELSE_PATTERN.match(line):
            return RobotInstruction(
                type=InstructionType.ELSE,
                line_number=line_num,
                raw_line=line
            )
        
        # Check for endif
        if self.ENDIF_PATTERN.match(line):
            return RobotInstruction(
                type=InstructionType.ENDIF,
                line_number=line_num,
                raw_line=line
            )
        
        # Unknown instruction
        self.errors.append(f"Line {line_num}: Unknown instruction: {line}")
        return RobotInstruction(
            type=InstructionType.UNKNOWN,
            line_number=line_num,
            raw_line=line
        )
    
    def get_errors(self) -> List[str]:
        """Return any parsing errors."""
        return self.errors


def validate_program(filepath: str) -> Tuple[bool, List[str]]:
    """Validate a program file and return (is_valid, errors)."""
    parser = ProgramParser()
    try:
        instructions = parser.parse_file(filepath)
        errors = parser.get_errors()
        
        # Check for unknown instructions
        unknown = [i for i in instructions if i.type == InstructionType.UNKNOWN]
        if unknown:
            errors.extend([f"Unknown instruction at line {i.line_number}" for i in unknown])
        
        # Check balanced if/endif blocks
        if_depth = 0
        for inst in instructions:
            if inst.type == InstructionType.IF:
                if_depth += 1
            elif inst.type == InstructionType.ENDIF:
                if_depth -= 1
                if if_depth < 0:
                    errors.append(f"Line {inst.line_number}: 'endif' without matching 'if'")
        if if_depth > 0:
            errors.append(f"Unmatched 'if' block(s): {if_depth} 'endif' missing")
        
        return len(errors) == 0, errors
    except FileNotFoundError:
        return False, [f"File not found: {filepath}"]
    except Exception as e:
        return False, [f"Error reading file: {e}"]


class PositionType(Enum):
    """Type of named position."""
    POSE = auto()   # Cartesian pose (x, y, z, qw, qx, qy, qz)
    JOINT = auto()  # Joint position (j1, j2, j3, j4, j5, j6)


@dataclass
class NamedPosition:
    """Represents a named robot position."""
    name: str
    position_type: PositionType
    # For POSE type: [x, y, z]
    position: Optional[List[float]] = None
    # For POSE type: [qw, qx, qy, qz]
    quaternion: Optional[List[float]] = None
    # For JOINT type: [j1, j2, j3, j4, j5, j6]
    joint_positions: Optional[List[float]] = None
    # Optional description/comment
    description: Optional[str] = None


class NamedPositionsParser:
    """
    Parses named positions configuration files.
    
    File format:
        # Comments start with #
        pose <name> <x> <y> <z> <qw> <qx> <qy> <qz>
        joint <name> <j1> <j2> <j3> <j4> <j5> <j6>
    
    Note: Joint angles in the config file should be in DEGREES.
          They will be automatically converted to radians when parsed.
    
    Example:
        pose Beaker 0.4 0.2 0.3 0.0 1.0 0.0 0.0
        joint Home 0.0 -90.0 90.0 -90.0 -90.0 0.0
    """
    
    # Regex patterns
    POSE_PATTERN = re.compile(
        r'^pose\s+(\S+)\s+'                    # pose <name>
        r'([-+]?\d*\.?\d+)\s+'                 # x
        r'([-+]?\d*\.?\d+)\s+'                 # y
        r'([-+]?\d*\.?\d+)\s+'                 # z
        r'([-+]?\d*\.?\d+)\s+'                 # qw
        r'([-+]?\d*\.?\d+)\s+'                 # qx
        r'([-+]?\d*\.?\d+)\s+'                 # qy
        r'([-+]?\d*\.?\d+)',                   # qz
        re.IGNORECASE
    )
    
    JOINT_PATTERN = re.compile(
        r'^joint\s+(\S+)\s+'                   # joint <name>
        r'([-+]?\d*\.?\d+)\s+'                 # j1
        r'([-+]?\d*\.?\d+)\s+'                 # j2
        r'([-+]?\d*\.?\d+)\s+'                 # j3
        r'([-+]?\d*\.?\d+)\s+'                 # j4
        r'([-+]?\d*\.?\d+)\s+'                 # j5
        r'([-+]?\d*\.?\d+)',                   # j6
        re.IGNORECASE
    )
    
    COMMENT_PATTERN = re.compile(r'^\s*#(.*)$')
    
    def __init__(self):
        self.positions: List[NamedPosition] = []
        self.errors: List[str] = []
    
    def parse_file(self, filepath: str) -> List[NamedPosition]:
        """Parse a named positions file and return list of positions."""
        self.positions = []
        self.errors = []
        
        try:
            with open(filepath, 'r') as f:
                lines = f.readlines()
        except FileNotFoundError:
            self.errors.append(f"File not found: {filepath}")
            return []
        except Exception as e:
            self.errors.append(f"Error reading file: {e}")
            return []
        
        current_comment = None
        
        for line_num, line in enumerate(lines, start=1):
            line = line.strip()
            
            # Skip empty lines
            if not line:
                current_comment = None
                continue
            
            # Check for comment
            comment_match = self.COMMENT_PATTERN.match(line)
            if comment_match:
                current_comment = comment_match.group(1).strip()
                continue
            
            # Try to parse as pose
            pose_match = self.POSE_PATTERN.match(line)
            if pose_match:
                try:
                    name = pose_match.group(1)
                    position = [
                        float(pose_match.group(2)),
                        float(pose_match.group(3)),
                        float(pose_match.group(4))
                    ]
                    quaternion = [
                        float(pose_match.group(5)),
                        float(pose_match.group(6)),
                        float(pose_match.group(7)),
                        float(pose_match.group(8))
                    ]
                    self.positions.append(NamedPosition(
                        name=name,
                        position_type=PositionType.POSE,
                        position=position,
                        quaternion=quaternion,
                        description=current_comment
                    ))
                    current_comment = None
                except ValueError as e:
                    self.errors.append(f"Line {line_num}: Failed to parse pose values: {e}")
                continue
            
            # Try to parse as joint
            joint_match = self.JOINT_PATTERN.match(line)
            if joint_match:
                try:
                    name = joint_match.group(1)
                    joints = [
                        float(joint_match.group(2)),
                        float(joint_match.group(3)),
                        float(joint_match.group(4)),
                        float(joint_match.group(5)),
                        float(joint_match.group(6)),
                        float(joint_match.group(7))
                    ]
                    # Convert from degrees to radians
                    joints_rad = [j * 3.14159265359 / 180.0 for j in joints]
                    self.positions.append(NamedPosition(
                        name=name,
                        position_type=PositionType.JOINT,
                        joint_positions=joints_rad,
                        description=current_comment
                    ))
                    current_comment = None
                except ValueError as e:
                    self.errors.append(f"Line {line_num}: Failed to parse joint values: {e}")
                continue
            
            # Unknown line format (not empty, not comment, not pose, not joint)
            # Just skip it silently unless it looks like an attempt at a command
            if line.lower().startswith(('pose', 'joint')):
                self.errors.append(f"Line {line_num}: Malformed position entry: {line}")
        
        return self.positions
    
    def get_errors(self) -> List[str]:
        """Return any parsing errors."""
        return self.errors
    
    def get_poses(self) -> List[NamedPosition]:
        """Return only pose-type positions."""
        return [p for p in self.positions if p.position_type == PositionType.POSE]
    
    def get_joints(self) -> List[NamedPosition]:
        """Return only joint-type positions."""
        return [p for p in self.positions if p.position_type == PositionType.JOINT]
    
    def get_by_name(self, name: str) -> Optional[NamedPosition]:
        """Get a position by name (case-insensitive)."""
        for p in self.positions:
            if p.name.lower() == name.lower():
                return p
        return None


if __name__ == '__main__':
    # Test the parser with extended instruction set
    test_program = """
# Extended program demo - shows new features
set_speed(0.5)

# Conditional: ensure gripper is open before starting
if gripper_closed
  opengripper
  wait(0.5)
endif

# Move to a named position
movetonamed(Home)
wait(1.0)

# Conditional: only go home if not already near it
if not_near(Home, 0.15)
  movetonamed(Home)
  wait(0.5)
endif

# Relative motion: nudge the end-effector
moverelative(left, 0.05)
wait(0.5)
moverelative(down, 0.03)
wait(0.5)

# Run a sub-program
runprogram(pick_resin_bottle.prog)

# Traditional commands still work
movetopose([0.4, 0.0, 0.5], [0.0, 1.0, 0.0, 0.0])
closegripper
wait(0.5)
opengripper
"""
    
    parser = ProgramParser()
    instructions = parser.parse_string(test_program)
    
    print("Parsed Instructions:")
    for inst in instructions:
        print(f"  Line {inst.line_number}: {inst.type.name}")
        if inst.pose:
            print(f"    Pose: pos={inst.pose[0]}, quat={inst.pose[1]}")
        if inst.joint_positions is not None:
            print(f"    Joints: {inst.joint_positions}")
        if inst.wait_duration is not None:
            print(f"    Duration: {inst.wait_duration}s")
        if inst.gripper_position is not None:
            print(f"    Gripper: {inst.gripper_position}")
        if inst.speed_factor is not None:
            print(f"    Speed: {inst.speed_factor}")
        if inst.named_position is not None:
            print(f"    Named: {inst.named_position}")
        if inst.relative_move is not None:
            print(f"    Relative: {inst.relative_move[0]} {inst.relative_move[1]}m from {inst.relative_move[2]}")
        if inst.sub_program is not None:
            print(f"    Sub-program: {inst.sub_program}")
        if inst.condition_type is not None:
            print(f"    Condition: {inst.condition_type} target={inst.condition_target} tol={inst.condition_tolerance}")
    
    if parser.get_errors():
        print("\nErrors:")
        for err in parser.get_errors():
            print(f"  {err}")
