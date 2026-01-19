#!/usr/bin/env python3
"""
Robot Program Parser for UR5 Control

Parses program files with robot instructions like:
  - movetopose([x, y, z], [qw, qx, qy, qz])
  - movetojoint([j1, j2, j3, j4, j5, j6])  # Joint angles in DEGREES
  - wait(seconds)
  - opengripper
  - closegripper
  - gripper(position)  # 0.0 = open, 1.0 = closed
  - # comments are ignored
  - set_speed(factor)  # 0.0-1.0 velocity scaling

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
    WAIT = auto()
    OPEN_GRIPPER = auto()
    CLOSE_GRIPPER = auto()
    GRIPPER = auto()
    SET_SPEED = auto()
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
    
    def __init__(self):
        self.instructions: List[RobotInstruction] = []
        self.errors: List[str] = []
    
    def parse_file(self, filepath: str) -> List[RobotInstruction]:
        """Parse a program file and return list of instructions."""
        self.instructions = []
        self.errors = []
        
        with open(filepath, 'r') as f:
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
    # Test the parser
    test_program = """
# Pick and place demo program
# Move to home position first
set_speed(0.5)
movetopose([0.4, 0.0, 0.5], [0.0, 1.0, 0.0, 0.0])
wait(1.0)

# Open gripper and approach object
opengripper
wait(0.5)
movetopose([0.4, 0.3, 0.2], [0.0, 0.707, 0.707, 0.0])

# Grasp object
closegripper
wait(0.5)

# Lift and move to place position
movetopose([0.4, 0.3, 0.4], [0.0, 0.707, 0.707, 0.0])
movetopose([0.4, -0.3, 0.4], [0.0, 0.707, 0.707, 0.0])

# Place object
movetopose([0.4, -0.3, 0.2], [0.0, 0.707, 0.707, 0.0])
opengripper
wait(0.5)

# Retreat
movetopose([0.4, -0.3, 0.4], [0.0, 0.707, 0.707, 0.0])
"""
    
    parser = ProgramParser()
    instructions = parser.parse_string(test_program)
    
    print("Parsed Instructions:")
    for inst in instructions:
        print(f"  Line {inst.line_number}: {inst.type.name}")
        if inst.pose:
            print(f"    Pose: pos={inst.pose[0]}, quat={inst.pose[1]}")
        if inst.wait_duration is not None:
            print(f"    Duration: {inst.wait_duration}s")
        if inst.gripper_position is not None:
            print(f"    Gripper: {inst.gripper_position}")
        if inst.speed_factor is not None:
            print(f"    Speed: {inst.speed_factor}")
    
    if parser.get_errors():
        print("\nErrors:")
        for err in parser.get_errors():
            print(f"  {err}")
