import open3d as o3d
import numpy as np
import argparse
import os
import copy
from bin_utils import read_bin_pointcloud, write_bin_pointcloud


class AutomaticAligner:
    def __init__(self, left_pcd_path, right_pcd_path, initial_transform=None):
        """
        Automatic point cloud alignment using feature matching and ICP
        
        Args:
            left_pcd_path: Path to left LiDAR point cloud
            right_pcd_path: Path to right LiDAR point cloud
            initial_transform: Initial transformation guess (4x4 matrix)
        """
        print("Loading point clouds...")
        self.left_path = left_pcd_path
        self.right_path = right_pcd_path
        
        # Load point clouds
        self.pcd_left, self.left_has_intensity, self.left_intensity = self._load_pointcloud(left_pcd_path)
        self.pcd_right_original, self.right_has_intensity, self.right_intensity = self._load_pointcloud(right_pcd_path)
        self.pcd_right = copy.deepcopy(self.pcd_right_original)
        
        print(f"Left LiDAR points: {len(self.pcd_left.points)}")
        print(f"Right LiDAR points: {len(self.pcd_right.points)}")
        
        # Initial transformation (24m along X-axis)
        if initial_transform is None:
            self.initial_transform = np.eye(4)
            self.initial_transform[0, 3] = -24.0  # Right is 24m along X from left
        else:
            self.initial_transform = initial_transform
        
        self.final_transform = None
        
    def _load_pointcloud(self, filepath):
        """Load point cloud from file"""
        ext = os.path.splitext(filepath)[1].lower()
        if ext == '.bin':
            return read_bin_pointcloud(filepath)
        else:
            pcd = o3d.io.read_point_cloud(filepath)
            return pcd, False, None
    
    def preprocess_point_cloud(self, pcd, voxel_size):
        """
        Preprocess point cloud: downsample and estimate normals
        
        Args:
            pcd: Input point cloud
            voxel_size: Voxel size for downsampling
            
        Returns:
            Downsampled point cloud with normals
        """
        print(f"  Downsampling with voxel size {voxel_size:.3f}")
        pcd_down = pcd.voxel_down_sample(voxel_size)
        
        print(f"  Estimating normals...")
        radius_normal = voxel_size * 2
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
        )
        
        return pcd_down
    
    def compute_fpfh_features(self, pcd, voxel_size):
        """
        Compute FPFH (Fast Point Feature Histogram) features
        
        Args:
            pcd: Point cloud with normals
            voxel_size: Voxel size for feature computation
            
        Returns:
            FPFH features
        """
        print(f"  Computing FPFH features...")
        radius_feature = voxel_size * 5
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
        )
        return fpfh
    
    def execute_global_registration(self, source, target, source_fpfh, target_fpfh, voxel_size):
        """
        Execute global registration using RANSAC
        
        Args:
            source: Source point cloud
            target: Target point cloud
            source_fpfh: Source FPFH features
            target_fpfh: Target FPFH features
            voxel_size: Voxel size
            
        Returns:
            Registration result
        """
        print("  Executing RANSAC-based global registration...")
        distance_threshold = voxel_size * 1.5
        
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source, target, source_fpfh, target_fpfh, 
            mutual_filter=True,
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=3,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
        )
        
        return result
    
    def execute_icp_registration(self, source, target, initial_transform, voxel_size, method='point_to_plane'):
        """
        Execute ICP (Iterative Closest Point) registration
        
        Args:
            source: Source point cloud
            target: Target point cloud
            initial_transform: Initial transformation
            voxel_size: Voxel size
            method: 'point_to_point' or 'point_to_plane'
            
        Returns:
            Registration result
        """
        print(f"  Executing ICP registration ({method})...")
        distance_threshold = voxel_size * 0.4
        
        if method == 'point_to_plane':
            result = o3d.pipelines.registration.registration_icp(
                source, target, distance_threshold, initial_transform,
                o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )
        else:  # point_to_point
            result = o3d.pipelines.registration.registration_icp(
                source, target, distance_threshold, initial_transform,
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
        
        return result
    
    def align_automatic(self, voxel_size=0.5, use_global=True, use_icp=True, icp_method='point_to_plane'):
        """
        Automatic alignment pipeline
        
        Args:
            voxel_size: Voxel size for downsampling (meters)
            use_global: Use global registration (RANSAC + features)
            use_icp: Use ICP for refinement
            icp_method: 'point_to_point' or 'point_to_plane'
            
        Returns:
            Final transformation matrix
        """
        print("\n" + "="*70)
        print("AUTOMATIC POINT CLOUD ALIGNMENT")
        print("="*70)
        
        # Start with initial transformation
        current_transform = self.initial_transform.copy()
        self.pcd_right = copy.deepcopy(self.pcd_right_original)
        self.pcd_right.transform(current_transform)
        
        print(f"\nInitial transformation applied:")
        print(current_transform)
        
        # Preprocessing
        print("\nPreprocessing point clouds...")
        pcd_left_down = self.preprocess_point_cloud(self.pcd_left, voxel_size)
        pcd_right_down = self.preprocess_point_cloud(self.pcd_right, voxel_size)
        print(f"  Left: {len(self.pcd_left.points)} -> {len(pcd_left_down.points)} points")
        print(f"  Right: {len(self.pcd_right.points)} -> {len(pcd_right_down.points)} points")
        
        # Global registration (optional)
        if use_global:
            print("\nPhase 1: Global Registration (RANSAC + FPFH)")
            print("-" * 70)
            
            # Compute features
            print("Computing features...")
            fpfh_left = self.compute_fpfh_features(pcd_left_down, voxel_size)
            fpfh_right = self.compute_fpfh_features(pcd_right_down, voxel_size)
            
            # Execute global registration
            result_global = self.execute_global_registration(
                pcd_right_down, pcd_left_down, 
                fpfh_right, fpfh_left, 
                voxel_size
            )
            
            print(f"\nGlobal registration result:")
            print(f"  Fitness: {result_global.fitness:.4f}")
            print(f"  RMSE: {result_global.inlier_rmse:.4f}")
            
            # Update transformation
            current_transform = result_global.transformation @ current_transform
            self.pcd_right.transform(result_global.transformation)
            pcd_right_down.transform(result_global.transformation)
        
        # ICP refinement (optional)
        if use_icp:
            print("\nPhase 2: ICP Refinement")
            print("-" * 70)
            
            # Execute ICP
            result_icp = self.execute_icp_registration(
                pcd_right_down, pcd_left_down,
                np.eye(4),  # Identity since already roughly aligned
                voxel_size,
                method=icp_method
            )
            
            print(f"\nICP registration result:")
            print(f"  Fitness: {result_icp.fitness:.4f}")
            print(f"  RMSE: {result_icp.inlier_rmse:.4f}")
            
            # Update final transformation
            current_transform = result_icp.transformation @ current_transform
            self.pcd_right.transform(result_icp.transformation)
        
        # Store final transformation
        self.final_transform = current_transform
        
        print("\n" + "="*70)
        print("Alignment complete!")
        print("="*70)
        print("\nFinal transformation matrix:")
        print(self.final_transform)
        
        return self.final_transform
    
    def evaluate_alignment(self, threshold=1.0):
        """
        Evaluate alignment quality
        
        Args:
            threshold: Distance threshold for fitness calculation
            
        Returns:
            Dictionary with evaluation metrics
        """
        print("\n" + "="*70)
        print("ALIGNMENT EVALUATION")
        print("="*70)
        
        # Compute correspondences
        result = o3d.pipelines.registration.evaluate_registration(
            self.pcd_right, self.pcd_left, threshold, np.eye(4)
        )
        
        metrics = {
            'fitness': result.fitness,
            'inlier_rmse': result.inlier_rmse,
            'threshold': threshold
        }
        
        print(f"\nAlignment Quality:")
        print(f"  Fitness: {metrics['fitness']:.4f} (higher is better, max=1.0)")
        print(f"  Inlier RMSE: {metrics['inlier_rmse']:.4f} meters")
        print(f"  Threshold: {threshold:.2f} meters")
        print("\nInterpretation:")
        if metrics['fitness'] > 0.7:
            print("  ✓ Excellent alignment")
        elif metrics['fitness'] > 0.5:
            print("  ✓ Good alignment")
        elif metrics['fitness'] > 0.3:
            print("  ⚠ Moderate alignment - may need manual adjustment")
        else:
            print("  ✗ Poor alignment - manual alignment recommended")
        
        print("="*70)
        
        return metrics
    
    def visualize_alignment(self):
        """Visualize the alignment result"""
        print("\nVisualizing alignment...")
        print("  Left LiDAR: RED")
        print("  Right LiDAR: BLUE")
        
        # Color point clouds
        pcd_left_vis = copy.deepcopy(self.pcd_left)
        pcd_right_vis = copy.deepcopy(self.pcd_right)
        pcd_left_vis.paint_uniform_color([1, 0, 0])   # Red
        pcd_right_vis.paint_uniform_color([0, 0, 1])  # Blue
        
        # Visualize
        o3d.visualization.draw_geometries(
            [pcd_left_vis, pcd_right_vis],
            window_name="Automatic Alignment Result",
            width=1280, height=720
        )
    
    def save_results(self, output_merged_path, output_transform_path):
        """Save aligned point cloud and transformation"""
        print("\nSaving results...")
        
        # Determine output format
        output_ext = os.path.splitext(output_merged_path)[1].lower()
        
        if output_ext == '.bin':
            # Save as .bin format
            left_points = np.asarray(self.pcd_left.points)
            right_points = np.asarray(self.pcd_right.points)
            merged_points = np.vstack([left_points, right_points])
            
            merged_pcd = o3d.geometry.PointCloud()
            merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
            
            if self.left_has_intensity and self.right_has_intensity:
                merged_intensity = np.hstack([self.left_intensity, self.right_intensity])
                write_bin_pointcloud(output_merged_path, merged_pcd, merged_intensity)
            else:
                write_bin_pointcloud(output_merged_path, merged_pcd, None)
        else:
            # Save as standard format
            merged_pcd = self.pcd_left + self.pcd_right
            o3d.io.write_point_cloud(output_merged_path, merged_pcd)
        
        print(f"Merged point cloud saved to: {output_merged_path}")
        
        # Save transformation matrix
        np.savetxt(output_transform_path, self.final_transform,
                   header="Automatic alignment transformation from right LiDAR to left LiDAR")
        print(f"Transformation matrix saved to: {output_transform_path}")
        
        # Save aligned right point cloud
        base, ext = os.path.splitext(output_merged_path)
        aligned_right_path = f"{base}_right_aligned{ext}"
        
        if ext == '.bin':
            if self.right_has_intensity:
                write_bin_pointcloud(aligned_right_path, self.pcd_right, self.right_intensity)
            else:
                write_bin_pointcloud(aligned_right_path, self.pcd_right, None)
        else:
            o3d.io.write_point_cloud(aligned_right_path, self.pcd_right)
        
        print(f"Aligned right point cloud saved to: {aligned_right_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Automatic LiDAR Point Cloud Alignment (Feature Matching + ICP)')
    parser.add_argument('--left', type=str, required=True,
                        help='Path to left LiDAR point cloud')
    parser.add_argument('--right', type=str, required=True,
                        help='Path to right LiDAR point cloud')
    parser.add_argument('--output', type=str, default='merged_auto_aligned.bin',
                        help='Output path for merged point cloud')
    parser.add_argument('--transform', type=str, default='transformation_auto.txt',
                        help='Output path for transformation matrix')
    parser.add_argument('--voxel-size', type=float, default=0.5,
                        help='Voxel size for downsampling (default: 0.5m)')
    parser.add_argument('--no-global', action='store_true',
                        help='Skip global registration (RANSAC)')
    parser.add_argument('--no-icp', action='store_true',
                        help='Skip ICP refinement')
    parser.add_argument('--icp-method', type=str, default='point_to_plane',
                        choices=['point_to_point', 'point_to_plane'],
                        help='ICP method (default: point_to_plane)')
    parser.add_argument('--initial-tx', type=float, default=-24.0,
                        help='Initial X translation guess (default: -24.0m)')
    parser.add_argument('--visualize', action='store_true',
                        help='Visualize alignment result')
    parser.add_argument('--evaluate', action='store_true',
                        help='Evaluate alignment quality')
    
    args = parser.parse_args()
    
    try:
        # Set initial transformation
        initial_transform = np.eye(4)
        initial_transform[0, 3] = args.initial_tx
        
        # Create aligner
        aligner = AutomaticAligner(args.left, args.right, initial_transform)
        
        # Run automatic alignment
        aligner.align_automatic(
            voxel_size=args.voxel_size,
            use_global=not args.no_global,
            use_icp=not args.no_icp,
            icp_method=args.icp_method
        )
        
        # Evaluate alignment
        if args.evaluate:
            aligner.evaluate_alignment(threshold=1.0)
        
        # Visualize
        if args.visualize:
            aligner.visualize_alignment()
        
        # Save results
        aligner.save_results(args.output, args.transform)
        
        print("\n✓ Automatic alignment complete!")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
