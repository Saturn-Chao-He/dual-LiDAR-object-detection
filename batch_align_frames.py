#!/usr/bin/env python3
"""
Batch Align LiDAR Frames

Apply a pre-calibrated transformation matrix to align and merge all frame pairs.
Saves merged results in .bin format.

Usage:
    python batch_align_frames.py --transform transformation.txt
"""

import numpy as np
import glob
import os
from pathlib import Path
from bin_utils import read_bin_pointcloud, write_bin_pointcloud


def batch_align_frames(left_dir='left', right_dir='right', 
                       output_dir='batch/merged', transform_file='transformation.txt'):
    """
    Batch align and merge all frame pairs from left and right directories
    
    Args:
        left_dir: Directory containing left LiDAR frames
        right_dir: Directory containing right LiDAR frames
        output_dir: Directory to save merged frames
        transform_file: Path to transformation matrix file
    """
    print("="*70)
    print("BATCH FRAME ALIGNMENT AND MERGING")
    print("="*70)
    
    # Load transformation matrix
    print(f"\n📂 Loading transformation matrix from: {transform_file}")
    if not os.path.exists(transform_file):
        print(f"❌ Error: Transformation file not found: {transform_file}")
        return
    
    transform = np.loadtxt(transform_file)
    print("✓ Transformation matrix loaded:")
    print(transform)
    
    # Create output directory
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    print(f"\n📂 Output directory: {output_dir}/")
    
    # Find all .bin files in left directory
    left_pattern = os.path.join(left_dir, '*.bin')
    left_files = sorted(glob.glob(left_pattern))
    
    if len(left_files) == 0:
        print(f"\n❌ No .bin files found in: {left_dir}/")
        print("   Make sure your left LiDAR frames are in the 'left' folder")
        return
    
    print(f"\n📊 Found {len(left_files)} frame pairs to process")
    print("="*70)
    
    # Statistics
    successful = 0
    failed = 0
    total_points_left = 0
    total_points_right = 0
    total_points_merged = 0
    
    # Process each frame pair
    for i, left_file in enumerate(left_files, 1):
        # Get filename
        left_basename = os.path.basename(left_file)
        frame_name = os.path.splitext(left_basename)[0]
        
        # Construct right file path
        right_file = os.path.join(right_dir, left_basename)
        
        # Check if right file exists
        if not os.path.exists(right_file):
            print(f"[{i}/{len(left_files)}] ⚠️  {left_basename}: Right file not found, skipping...")
            failed += 1
            continue
        
        try:
            # Load left point cloud
            pcd_left, left_has_intensity, left_intensity = read_bin_pointcloud(left_file)
            num_left = len(pcd_left.points)
            
            # Load right point cloud
            pcd_right, right_has_intensity, right_intensity = read_bin_pointcloud(right_file)
            num_right = len(pcd_right.points)
            
            # Apply transformation to right point cloud
            pcd_right.transform(transform)
            
            # Merge point clouds
            left_points = np.asarray(pcd_left.points)
            right_points = np.asarray(pcd_right.points)
            merged_points = np.vstack([left_points, right_points])
            num_merged = len(merged_points)
            
            # Create merged point cloud
            from open3d.geometry import PointCloud
            from open3d.utility import Vector3dVector
            merged_pcd = PointCloud()
            merged_pcd.points = Vector3dVector(merged_points)
            
            # Merge intensities if available
            if left_has_intensity and right_has_intensity:
                merged_intensity = np.hstack([left_intensity, right_intensity])
            else:
                merged_intensity = None
            
            # Save merged point cloud
            output_file = os.path.join(output_dir, f"merged_{left_basename}")
            write_bin_pointcloud(output_file, merged_pcd, merged_intensity)
            
            # Update statistics
            successful += 1
            total_points_left += num_left
            total_points_right += num_right
            total_points_merged += num_merged
            
            # Print progress
            print(f"[{i}/{len(left_files)}] ✓ {left_basename}: "
                  f"L:{num_left:,} + R:{num_right:,} → {num_merged:,} pts")
            
        except Exception as e:
            print(f"[{i}/{len(left_files)}] ❌ {left_basename}: Error - {str(e)}")
            failed += 1
            continue
    
    # Final summary
    print("="*70)
    print("BATCH PROCESSING COMPLETE")
    print("="*70)
    print(f"\n📊 Results:")
    print(f"   Total frames processed:  {len(left_files)}")
    print(f"   ✓ Successful:            {successful}")
    print(f"   ❌ Failed:                {failed}")
    
    if successful > 0:
        print(f"\n📈 Point Statistics:")
        print(f"   Average left points:     {total_points_left // successful:,}")
        print(f"   Average right points:    {total_points_right // successful:,}")
        print(f"   Average merged points:   {total_points_merged // successful:,}")
        print(f"   Point increase:          {(total_points_merged / (total_points_left + 1e-10) - 1) * 100:.1f}%")
    
    print(f"\n📂 Output location:")
    print(f"   {os.path.abspath(output_dir)}/")
    print(f"   Files: merged_*.bin")
    
    print("\n" + "="*70)
    print("✓ All frames aligned and saved!")
    print("="*70)
    
    return successful, failed


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Batch align and merge LiDAR frames using pre-calibrated transformation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage (with default folders)
  python batch_align_frames.py --transform transformation.txt
  
  # Custom folders
  python batch_align_frames.py \\
      --transform calibration/transformation.txt \\
      --left-dir data/left \\
      --right-dir data/right \\
      --output-dir results/merged

Folder Structure:
  left/
    ├── 000000.bin
    ├── 000001.bin
    └── ...
  right/
    ├── 000000.bin
    ├── 000001.bin
    └── ...
  
  After processing:
  batch/merged/
    ├── merged_000000.bin
    ├── merged_000001.bin
    └── ...
        """)
    
    parser.add_argument('--transform', type=str, required=True,
                       help='Path to transformation matrix file (required)')
    parser.add_argument('--left-dir', type=str, default='left',
                       help='Directory with left LiDAR frames (default: left)')
    parser.add_argument('--right-dir', type=str, default='right',
                       help='Directory with right LiDAR frames (default: right)')
    parser.add_argument('--output-dir', type=str, default='batch/merged',
                       help='Output directory for merged frames (default: batch/merged)')
    
    args = parser.parse_args()
    
    # Run batch alignment
    try:
        successful, failed = batch_align_frames(
            left_dir=args.left_dir,
            right_dir=args.right_dir,
            output_dir=args.output_dir,
            transform_file=args.transform
        )
        
        if successful > 0:
            print(f"\n✓ Success! {successful} frames processed and saved.")
            print(f"\n💡 Next steps:")
            print(f"   Use merged frames for object detection and tracking")
            print(f"   Expect better accuracy with ~2x point density!")
        else:
            print(f"\n⚠️  No frames were successfully processed.")
            print(f"   Check that your folders contain matching .bin files.")
            
    except KeyboardInterrupt:
        print("\n\n⚠️  Process interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()