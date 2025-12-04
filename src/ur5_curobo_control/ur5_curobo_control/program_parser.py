#!/usr/bin/env python3
"""
Robot Program Parser for UR5 Control

Parses program files with robot instructions like:
  - movetopose([x, y, z], [qw, qx, qy, qz])
  - movetojoint([j1, j2, j3, j4, j5, j6])
  - wait(seconds)
  - opengripper
  - closegripper
  - gripper(position)  # 0.0 = open, 1.0 = closed
  - # comments are ignored
  - set_speed(factor)  # 0.0-1.0 velocity scaling
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
                return RobotInstruction(
                    type=InstructionType.MOVE_TO_JOINT,
                    line_number=line_num,
                    raw_line=line,
                    joint_positions=joints
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
