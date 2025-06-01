#!/usr/bin/python3
# coding=utf8
import numpy as np

class BlockTracker:
    """
    Tracks blocks and manages their state.
    """
    def __init__(self):
        self.blocks = {}  # Dictionary to store block information
        self.picked_blocks = set()  # Set of picked block IDs
    
    def update_block_positions(self, frame_lab, lab_data, target_colors, camera_processor):
        """
        Update block positions using camera processor.
        
        Args:
            frame_lab: Frame in LAB color space
            lab_data: LAB color range data
            target_colors: List of target colors to detect
            camera_processor: CameraProcessor instance
            
        Returns:
            dict: Dictionary of detected blocks
        """
        # Detect blocks using camera processor
        detected_blocks = camera_processor.detect_blocks(frame_lab, lab_data, target_colors)
        
        # Update block information
        self.blocks = detected_blocks
        return detected_blocks
    
    def get_next_block(self, target_colors):
        """
        Get the next block to pick up based on target colors.
        
        Args:
            target_colors: List of target colors in order
            
        Returns:
            tuple: (block_id, block_data) or (None, None)
        """
        # Find the first unpicked block of the target color
        for color in target_colors:
            for block_id, block_data in self.blocks.items():
                if (block_id not in self.picked_blocks and 
                    block_data['color'] == color):
                    return block_id, block_data
        
        return None, None
    
    def mark_block_as_picked(self, block_id):
        """
        Mark a block as picked.
        
        Args:
            block_id: ID of the block to mark
        """
        self.picked_blocks.add(block_id) 