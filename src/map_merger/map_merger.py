import cv2
import numpy as np
import yaml
import os

def get_workspace_path():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(current_dir))
    return workspace_dir

def load_map_from_yaml(yaml_file):
    workspace_path = get_workspace_path()
    yaml_path = os.path.join(workspace_path, yaml_file)
    
    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
        
    pgm_file = yaml_path.replace('.yaml', '.pgm')
    map_img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
    
    if map_img is None:
        raise ValueError(f"Failed to load map image from {pgm_file}")
    
    return map_img, map_data

def find_center_line(map_img):
    """Find the vertical center line of the explored area"""
    # Find where the map has data (not unknown space)
    explored = map_img != 205
    cols = np.any(explored, axis=0)
    explored_cols = np.where(cols)[0]
    if len(explored_cols) > 0:
        return (explored_cols[0] + explored_cols[-1]) // 2
    return map_img.shape[1] // 2

def merge_hallway_maps(map1_yaml, map2_yaml):
    """Merge two hallway maps with overlap in the middle"""
    # Load maps
    map1_img, map1_data = load_map_from_yaml(map1_yaml)
    map2_img, map2_data = load_map_from_yaml(map2_yaml)
    
    print(f"Loaded maps - Map1: {map1_img.shape}, Map2: {map2_img.shape}")
    
    # Find center lines
    center1 = find_center_line(map1_img)
    center2 = find_center_line(map2_img)
    
    print(f"Found centers - Map1: {center1}, Map2: {center2}")
    
    # Calculate total width needed
    total_width = map1_img.shape[1] + map2_img.shape[1] - center1 - center2
    total_height = max(map1_img.shape[0], map2_img.shape[0])
    
    # Create merged map
    merged_map = np.full((total_height, total_width), 205, dtype=np.uint8)
    
    # Copy first map
    h1, w1 = map1_img.shape
    merged_map[0:h1, 0:w1] = map1_img
    
    # Copy second map with offset
    h2, w2 = map2_img.shape
    offset_x = map1_img.shape[1] - center1 - center2
    
    # Create masks for blending
    overlap_start = max(0, offset_x)
    overlap_end = min(offset_x + w2, total_width)
    overlap_width = overlap_end - overlap_start
    
    if overlap_width > 0:
        # Calculate blending weights
        x = np.linspace(0, 1, overlap_width)
        alpha = x.reshape(1, -1)
        
        # Extract overlapping regions
        region1 = merged_map[0:h2, overlap_start:overlap_end]
        region2 = map2_img[:, 0:overlap_width]
        
        # Blend only where both maps have data (not unknown space)
        mask1 = region1 != 205
        mask2 = region2 != 205
        
        # Where both maps have data, use minimum value (darker color)
        overlap_region = np.where(mask1 & mask2, 
                                np.minimum(region1, region2),
                                np.where(mask1, region1, region2))
        
        merged_map[0:h2, overlap_start:overlap_end] = overlap_region
    
    # Copy non-overlapping part of second map
    if overlap_end < total_width:
        merged_map[0:h2, overlap_end:min(overlap_end + w2 - overlap_width, total_width)] = \
            map2_img[:, overlap_width:]
    
    return merged_map, map1_data

def save_merged_map(merged_map, merged_data, output_prefix='merged_map'):
    workspace_path = get_workspace_path()
    output_path = os.path.join(workspace_path, output_prefix)
    
    # Save original merged map
    cv2.imwrite(f'{output_path}.pgm', merged_map)
    print(f"Saved merged map to {output_path}.pgm")
    
    # Save enhanced version for visualization
    enhanced_map = cv2.normalize(merged_map, None, 0, 255, cv2.NORM_MINMAX)
    cv2.imwrite(f'{output_path}_enhanced.pgm', enhanced_map)
    print(f"Saved enhanced map to {output_path}_enhanced.pgm")
    
    with open(f'{output_path}.yaml', 'w') as f:
        yaml.dump(merged_data, f)
    print(f"Saved merged map config to {output_path}.yaml")

if __name__ == '__main__':
    try:
        # Merge maps
        merged_map, merged_data = merge_hallway_maps('carter1_map.yaml', 'carter2_map.yaml')
        
        # Save results
        save_merged_map(merged_map, merged_data)
        
        # Display result
        cv2.imshow('Merged Map', merged_map)
        print("Press any key to close the window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        # Also display enhanced version
        enhanced_map = cv2.normalize(merged_map, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow('Enhanced Merged Map', enhanced_map)
        print("Press any key to close the window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    except Exception as e:
        print(f"Error: {e}")