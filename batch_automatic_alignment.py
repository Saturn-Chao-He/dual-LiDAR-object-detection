#!/usr/bin/env python3
"""
Batch Automatic Alignment with Frame Saving

Process multiple frame pairs automatically using feature matching + ICP
Saves all aligned merged frames to output folder
"""

import os
import glob
import argparse
import numpy as np
from pathlib import Path
from automatic_alignment import AutomaticAligner


def batch_automatic_alignment(left_dir, right_dir, output_dir, 
                               voxel_size=0.5, file_pattern='*.bin',
                               save_frames=True, min_fitness=0.5):
    """
    Batch process multiple frame pairs with automatic alignment
    
    Args:
        left_dir: Directory with left LiDAR frames
        right_dir: Directory with right LiDAR frames
        output_dir: Directory to save results
        voxel_size: Voxel size for downsampling
        file_pattern: File pattern to match
        save_frames: Save aligned merged frames
        min_fitness: Minimum fitness score to accept
    """
    # Create output directories
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    if save_frames:
        frames_dir = os.path.join(output_dir, 'aligned_frames')
        Path(frames_dir).mkdir(parents=True, exist_ok=True)
    
    # Find all left frames
    left_files = sorted(glob.glob(os.path.join(left_dir, file_pattern)))
    
    if len(left_files) == 0:
        print(f"No files found matching {file_pattern} in {left_dir}")
        return None, None
    
    print("="*70)
    print(f"BATCH AUTOMATIC ALIGNMENT")
    print("="*70)
    print(f"Found {len(left_files)} frame pairs")
    print(f"Voxel size: {voxel_size}m")
    print(f"Minimum fitness: {min_fitness}")
    print(f"Save frames: {save_frames}")
    print("="*70 + "\n")
    
    results = []
    transformations = []
    
    # Process each frame pair
    for i, left_file in enumerate(left_files, 1):
        left_basename = os.path.basename(left_file)
        right_file = os.path.join(right_dir, left_basename)
        
        if not os.path.exists(right_file):
            print(f"[{i}/{len(left_files)}] ✗ Skipping {left_basename} - right file not found")
            continue
        
        print(f"[{i}/{len(left_files)}] Processing {left_basename}...")
        
        try:
            # Create aligner
            aligner = AutomaticAligner(left_file, right_file)
            
            # Run automatic alignment
            transform = aligner.align_automatic(
                voxel_size=voxel_size,
                use_global=True,
                use_icp=True,
                icp_method='point_to_plane'
            )
            
            # Evaluate
            metrics = aligner.evaluate_alignment(threshold=1.0)
            
            # Save aligned merged frame if requested
            if save_frames:
                output_frame = os.path.join(frames_dir, f"aligned_{left_basename}")
                # Save only the merged frame directly
                from bin_utils import write_bin_pointcloud
                
                # Get points from both clouds
                left_points = np.asarray(aligner.pcd_left.points)
                right_points = np.asarray(aligner.pcd_right.points)
                merged_points = np.vstack([left_points, right_points])
                
                # Create merged point cloud
                import open3d as o3d
                merged_pcd = o3d.geometry.PointCloud()
                merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
                
                # Merge intensities if available
                if aligner.left_has_intensity and aligner.right_has_intensity:
                    merged_intensity = np.hstack([aligner.left_intensity, aligner.right_intensity])
                else:
                    merged_intensity = None
                
                # Save merged frame
                write_bin_pointcloud(output_frame, merged_pcd, merged_intensity)
            
            # Store results
            result = {
                'filename': left_basename,
                'fitness': metrics['fitness'],
                'rmse': metrics['inlier_rmse'],
                'transform': transform,
            }
            results.append(result)
            transformations.append(transform)
            
            # Print summary
            status = "✓" if metrics['fitness'] >= min_fitness else "⚠"
            print(f"  {status} Fitness: {metrics['fitness']:.4f}, RMSE: {metrics['inlier_rmse']:.4f}m")
            if save_frames:
                print(f"  → Saved: {output_frame}")
            print()
            
        except Exception as e:
            print(f"  ✗ Error: {e}\n")
            import traceback
            traceback.print_exc()
            continue
    
    # Summary
    print("\n" + "="*70)
    print("BATCH PROCESSING COMPLETE")
    print("="*70)
    
    if len(results) == 0:
        print("No frames were successfully processed.")
        return None, None
    
    # Calculate statistics
    fitnesses = [r['fitness'] for r in results]
    rmses = [r['rmse'] for r in results]
    
    print(f"\nProcessed: {len(results)}/{len(left_files)} frames")
    print(f"\nFitness Statistics:")
    print(f"  Mean: {np.mean(fitnesses):.4f}")
    print(f"  Std:  {np.std(fitnesses):.4f}")
    print(f"  Min:  {np.min(fitnesses):.4f}")
    print(f"  Max:  {np.max(fitnesses):.4f}")
    
    print(f"\nRMSE Statistics:")
    print(f"  Mean: {np.mean(rmses):.4f}m")
    print(f"  Std:  {np.std(rmses):.4f}m")
    print(f"  Min:  {np.min(rmses):.4f}m")
    print(f"  Max:  {np.max(rmses):.4f}m")
    
    # Quality assessment
    good_alignments = sum(1 for f in fitnesses if f >= 0.6)
    moderate_alignments = sum(1 for f in fitnesses if 0.4 <= f < 0.6)
    poor_alignments = sum(1 for f in fitnesses if f < 0.4)
    
    print(f"\nQuality Assessment:")
    print(f"  Good (fitness ≥ 0.6):     {good_alignments}")
    print(f"  Moderate (0.4-0.6):       {moderate_alignments}")
    print(f"  Poor (< 0.4):             {poor_alignments}")
    
    # Find best transformation
    best_idx = np.argmax(fitnesses)
    best_result = results[best_idx]
    
    print(f"\nBest Alignment:")
    print(f"  Frame: {best_result['filename']}")
    print(f"  Fitness: {best_result['fitness']:.4f}")
    print(f"  RMSE: {best_result['rmse']:.4f}m")
    
    # Save ONLY the best transformation (not individual ones)
    best_transform_path = os.path.join(output_dir, 'transformation_best.txt')
    np.savetxt(best_transform_path, best_result['transform'],
               header=f"Best transformation (from {best_result['filename']}, fitness={best_result['fitness']:.4f})")
    print(f"\n✓ Best transformation saved to: {best_transform_path}")
    
    # Also compute average transformation (for reference)
    avg_transform = np.mean(transformations, axis=0)
    avg_transform_path = os.path.join(output_dir, 'transformation_average.txt')
    np.savetxt(avg_transform_path, avg_transform,
               header=f"Average transformation from {len(transformations)} frames")
    print(f"✓ Average transformation saved to: {avg_transform_path}")
    
    if save_frames:
        print(f"\n✓ All {len(results)} aligned frames saved to: {frames_dir}/")
    
    print("\n" + "="*70)
    
    return results, best_result


def compare_with_manual(automatic_transform, manual_transform):
    """
    Compare automatic alignment with manual alignment
    
    Args:
        automatic_transform: Path to automatic transformation
        manual_transform: Path to manual transformation
    """
    print("\n" + "="*70)
    print("COMPARING AUTOMATIC vs MANUAL ALIGNMENT")
    print("="*70)
    
    # Load transformations
    T_auto = np.loadtxt(automatic_transform)
    T_manual = np.loadtxt(manual_transform)
    
    # Extract translation
    t_auto = T_auto[:3, 3]
    t_manual = T_manual[:3, 3]
    
    # Extract rotation (convert to Euler angles)
    from scipy.spatial.transform import Rotation
    R_auto = T_auto[:3, :3]
    R_manual = T_manual[:3, :3]
    
    euler_auto = Rotation.from_matrix(R_auto).as_euler('xyz', degrees=True)
    euler_manual = Rotation.from_matrix(R_manual).as_euler('xyz', degrees=True)
    
    # Calculate differences
    translation_diff = np.linalg.norm(t_auto - t_manual)
    rotation_diff = np.linalg.norm(euler_auto - euler_manual)
    
    print(f"\nTranslation:")
    print(f"  Automatic: [{t_auto[0]:.3f}, {t_auto[1]:.3f}, {t_auto[2]:.3f}]")
    print(f"  Manual:    [{t_manual[0]:.3f}, {t_manual[1]:.3f}, {t_manual[2]:.3f}]")
    print(f"  Difference: {translation_diff:.3f}m")
    
    print(f"\nRotation (degrees):")
    print(f"  Automatic: [{euler_auto[0]:.2f}, {euler_auto[1]:.2f}, {euler_auto[2]:.2f}]")
    print(f"  Manual:    [{euler_manual[0]:.2f}, {euler_manual[1]:.2f}, {euler_manual[2]:.2f}]")
    print(f"  Difference: {rotation_diff:.2f}°")
    
    print(f"\nAssessment:")
    if translation_diff < 0.1 and rotation_diff < 2:
        print("  ✓ Excellent agreement - transformations are very similar")
    elif translation_diff < 0.3 and rotation_diff < 5:
        print("  ✓ Good agreement - transformations are similar")
    elif translation_diff < 0.5 and rotation_diff < 10:
        print("  ⚠ Moderate agreement - some differences exist")
    else:
        print("  ✗ Poor agreement - significant differences")
        print("    → Manual alignment may be more accurate")
    
    print("="*70)


def main():
    parser = argparse.ArgumentParser(
        description='Batch Automatic Alignment with Aligned Frame Saving',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process multiple frames and save aligned results
  python batch_automatic_alignment.py batch \\
      --left-dir left \\
      --right-dir right \\
      --output-dir results
  
  # Custom voxel size
  python batch_automatic_alignment.py batch \\
      --left-dir left --right-dir right \\
      --output-dir results --voxel-size 0.3
  
  # Don't save frames (only get best transformation)
  python batch_automatic_alignment.py batch \\
      --left-dir left --right-dir right \\
      --output-dir results --no-save-frames

Output Structure:
  results/
  ├── transformation_best.txt      ← Best transformation
  ├── transformation_average.txt   ← Average transformation
  └── aligned_frames/              ← All aligned merged frames
      ├── aligned_000000.bin
      ├── aligned_000001.bin
      └── ...
        """)
    
    subparsers = parser.add_subparsers(dest='command', help='Command to run')
    
    # Batch processing
    batch_parser = subparsers.add_parser('batch', help='Batch process frames')
    batch_parser.add_argument('--left-dir', type=str, required=True,
                             help='Directory with left LiDAR frames')
    batch_parser.add_argument('--right-dir', type=str, required=True,
                             help='Directory with right LiDAR frames')
    batch_parser.add_argument('--output-dir', type=str, default='./auto_calibration',
                             help='Output directory (default: ./auto_calibration)')
    batch_parser.add_argument('--voxel-size', type=float, default=0.5,
                             help='Voxel size (default: 0.5m)')
    batch_parser.add_argument('--pattern', type=str, default='*.bin',
                             help='File pattern (default: *.bin)')
    batch_parser.add_argument('--min-fitness', type=float, default=0.5,
                             help='Minimum fitness score (default: 0.5)')
    batch_parser.add_argument('--no-save-frames', action='store_true',
                             help='Do not save aligned frames (only save best transformation)')
    
    # Comparison
    compare_parser = subparsers.add_parser('compare', 
                                          help='Compare automatic vs manual')
    compare_parser.add_argument('--automatic', type=str, required=True,
                               help='Path to automatic transformation')
    compare_parser.add_argument('--manual', type=str, required=True,
                               help='Path to manual transformation')
    
    args = parser.parse_args()
    
    if args.command == 'batch':
        results, best = batch_automatic_alignment(
            args.left_dir,
            args.right_dir,
            args.output_dir,
            voxel_size=args.voxel_size,
            file_pattern=args.pattern,
            save_frames=not args.no_save_frames,
            min_fitness=args.min_fitness
        )
        
        if best:
            print(f"\n💡 Usage Tips:")
            if not args.no_save_frames:
                print(f"   1. All aligned frames are in: {args.output_dir}/aligned_frames/")
                print(f"   2. Use these frames directly in your detection/tracking pipeline")
            print(f"   3. Best transformation: {args.output_dir}/transformation_best.txt")
            print(f"   4. Use best transformation to align more frames:")
            print(f"      python batch_align_frames.py \\")
            print(f"          --transform {args.output_dir}/transformation_best.txt \\")
            print(f"          --left-dir {args.left_dir} \\")
            print(f"          --right-dir {args.right_dir} \\")
            print(f"          --output-dir batch/merged")
    
    elif args.command == 'compare':
        try:
            compare_with_manual(args.automatic, args.manual)
        except ImportError:
            print("Error: scipy is required for comparison")
            print("Install with: pip install scipy")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()