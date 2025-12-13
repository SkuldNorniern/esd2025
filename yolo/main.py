"""
YOLO Training Script for Ball Position Tracking

This script provides a flexible interface for training YOLO models on custom datasets,
specifically configured for ball detection and position tracking.
Supports various training configurations including device selection, hyperparameters, and logging.

For ball tracking, the model will detect ball position (center_x, center_y) and size,
which can be used for real-time tracking in the laser turret system.
"""

import argparse
import os
from pathlib import Path
from ultralytics import YOLO


def parse_args():
    """Parse command line arguments for training configuration."""
    parser = argparse.ArgumentParser(
        description="Train YOLO model on custom dataset",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Model configuration
    parser.add_argument(
        "--model",
        type=str,
        default="yolo11n.pt",
        help="Path to model file (.pt for pretrained, .yaml for from scratch)"
    )
    parser.add_argument(
        "--pretrained",
        type=str,
        default=None,
        help="Path to pretrained weights (when using .yaml model)"
    )
    
    # Dataset configuration
    parser.add_argument(
        "--data",
        type=str,
        default=None,
        help="Path to dataset configuration YAML file (e.g., ball_dataset.yaml). Required unless using --roboflow."
    )
    
    # Roboflow dataset configuration
    parser.add_argument(
        "--roboflow",
        action="store_true",
        help="Download dataset from Roboflow (requires --roboflow-url or --roboflow-workspace/--roboflow-project)"
    )
    parser.add_argument(
        "--roboflow-url",
        type=str,
        default=None,
        help="Roboflow dataset URL (e.g., https://universe.roboflow.com/workspace/project)"
    )
    parser.add_argument(
        "--roboflow-workspace",
        type=str,
        default=None,
        help="Roboflow workspace name (e.g., homework2warnerhoefakker)"
    )
    parser.add_argument(
        "--roboflow-project",
        type=str,
        default=None,
        help="Roboflow project name (e.g., tennis-kfdyt or tennis)"
    )
    parser.add_argument(
        "--roboflow-version",
        type=str,
        default=None,
        help="Roboflow dataset version number (e.g., 1, 2, 3) or 'latest' (defaults to latest)"
    )
    parser.add_argument(
        "--roboflow-api-key",
        type=str,
        default=None,
        help="Roboflow API key (or set ROBOFLOW_API_KEY environment variable)"
    )
    
    # Training hyperparameters
    parser.add_argument(
        "--epochs",
        type=int,
        default=100,
        help="Number of training epochs"
    )
    parser.add_argument(
        "--batch",
        type=int,
        default=16,
        help="Batch size for training"
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Image size for training (height and width)"
    )
    parser.add_argument(
        "--lr0",
        type=float,
        default=0.01,
        help="Initial learning rate"
    )
    parser.add_argument(
        "--lrf",
        type=float,
        default=0.01,
        help="Final learning rate (lr0 * lrf)"
    )
    
    # Device configuration
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Device to use for training (e.g., 'cpu', '0', '0,1', 'mps', '-1' for idle GPU)"
    )
    
    # Training options
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume training from last checkpoint"
    )
    parser.add_argument(
        "--project",
        type=str,
        default="runs/train",
        help="Project directory for saving training results"
    )
    parser.add_argument(
        "--name",
        type=str,
        default=None,
        help="Experiment name (defaults to timestamp)"
    )
    parser.add_argument(
        "--save",
        action="store_true",
        default=True,
        help="Save training checkpoints and final model"
    )
    parser.add_argument(
        "--save-period",
        type=int,
        default=-1,
        help="Save checkpoint every N epochs (-1 to disable)"
    )
    
    # Logging options
    parser.add_argument(
        "--loggers",
        type=str,
        nargs="*",
        choices=["tensorboard", "comet", "clearml"],
        help="Logging backends to use (tensorboard, comet, clearml)"
    )
    
    return parser.parse_args()


def download_roboflow_dataset(
    url: str | None = None,
    workspace: str | None = None,
    project: str | None = None,
    version: str | None = None,
    api_key: str | None = None
) -> str | None:
    """
    Download dataset from Roboflow and return path to dataset YAML file.
    
    Args:
        url: Full Roboflow URL (e.g., https://universe.roboflow.com/workspace/project)
        workspace: Roboflow workspace name
        project: Roboflow project name
        version: Dataset version number or 'latest' (defaults to latest)
        api_key: Roboflow API key (or from ROBOFLOW_API_KEY env var)
    
    Returns:
        Path to the dataset YAML file, or None if download failed
    """
    try:
        from roboflow import Roboflow
    except ImportError:
        print("Error: roboflow package not installed. Install with: pip install roboflow")
        return None
    
    # Get API key from argument or environment
    if api_key is None:
        api_key = os.getenv("ROBOFLOW_API_KEY")
        if api_key is None:
            print("Error: Roboflow API key required. Set --roboflow-api-key or ROBOFLOW_API_KEY environment variable")
            print("Get your API key from: https://app.roboflow.com/")
            return None
    
    try:
        rf = Roboflow(api_key=api_key)
        
        # Parse URL if provided
        if url:
            # Extract workspace and project from URL
            # Format: https://universe.roboflow.com/workspace/project
            url_clean = url.rstrip('/')
            # Handle both full URLs and relative paths
            if 'universe.roboflow.com' in url_clean or 'app.roboflow.com' in url_clean:
                parts = url_clean.split('/')
                # Find the workspace and project after the domain
                for i, part in enumerate(parts):
                    if 'roboflow.com' in part and i + 2 < len(parts):
                        workspace = parts[i + 1] if not workspace else workspace
                        project = parts[i + 2] if not project else project
                        break
            else:
                # Simple path format: workspace/project
                parts = url_clean.split('/')
                if len(parts) >= 2:
                    workspace = parts[-2] if not workspace else workspace
                    project = parts[-1] if not project else project
        
        if not workspace or not project:
            print("Error: Could not determine workspace and project. Provide --roboflow-url or --roboflow-workspace/--roboflow-project")
            return None
        
        print(f"Connecting to Roboflow workspace: {workspace}")
        print(f"Loading project: {project}")
        project_obj = rf.workspace(workspace).project(project)
        
        # Determine which version to download
        if version is None or version.lower() == "latest":
            # Get the latest version number
            versions = project_obj.versions()
            if not versions:
                print("Error: No versions found for this project")
                return None
            
            # Extract version numbers from Version objects
            # Roboflow Version objects typically have a 'version' attribute or can be converted
            version_numbers = []
            for v in versions:
                try:
                    # Try common attributes first
                    if hasattr(v, 'version'):
                        version_num = int(v.version)
                    elif hasattr(v, 'id'):
                        version_num = int(v.id)
                    elif hasattr(v, 'name'):
                        # Sometimes version is in name like "1", "2", etc.
                        version_num = int(v.name)
                    else:
                        # Try string conversion
                        version_str = str(v).strip()
                        version_num = int(version_str)
                    version_numbers.append(version_num)
                except (ValueError, TypeError, AttributeError) as e:
                    # Skip versions we can't parse
                    print(f"Warning: Could not parse version {v}: {e}")
                    continue
            
            if not version_numbers:
                print("Error: Could not extract version numbers from project versions")
                print(f"Available versions (type: {type(versions[0]) if versions else 'N/A'}): {versions[:3]}")
                return None
            
            latest_version_num = max(version_numbers)
            print(f"Found {len(version_numbers)} version(s), downloading latest: {latest_version_num}")
            dataset = project_obj.version(latest_version_num).download("yolov8")
            version_num = latest_version_num
        else:
            print(f"Downloading version {version}...")
            dataset = project_obj.version(int(version)).download("yolov8")
            version_num = int(version)
        
        # Roboflow downloads to a directory and creates a data.yaml file
        # The location might be a string or Path object
        if hasattr(dataset, 'location'):
            dataset_path = Path(dataset.location)
        elif hasattr(dataset, 'path'):
            dataset_path = Path(dataset.path)
        else:
            # Try to find the dataset in common locations
            possible_paths = [
                Path(f"{project}-{version_num}"),
                Path(f"./{project}"),
                Path(f"./datasets/{project}")
            ]
            dataset_path = None
            for p in possible_paths:
                if p.exists() and (p / "data.yaml").exists():
                    dataset_path = p
                    break
            
            if dataset_path is None:
                print("Error: Could not determine dataset download location")
                return None
        
        yaml_path = dataset_path / "data.yaml"
        
        if yaml_path.exists():
            print(f"Dataset downloaded successfully to: {dataset_path}")
            print(f"Dataset YAML: {yaml_path}")
            return str(yaml_path)
        else:
            print(f"Warning: data.yaml not found at {yaml_path}")
            # Try to find it in the directory
            yaml_files = list(dataset_path.glob("*.yaml"))
            if yaml_files:
                print(f"Found YAML file: {yaml_files[0]}")
                return str(yaml_files[0])
            print(f"Available files in {dataset_path}: {list(dataset_path.iterdir())}")
            return None
            
    except Exception as e:
        print(f"Error downloading Roboflow dataset: {e}")
        print("Make sure your API key is correct and you have access to the dataset")
        import traceback
        traceback.print_exc()
        return None


def validate_dataset_config(data_path: str) -> bool:
    """Validate that the dataset configuration file exists."""
    path = Path(data_path)
    if not path.exists():
        print(f"Error: Dataset configuration file not found: {data_path}")
        return False
    if not path.suffix == ".yaml":
        print(f"Warning: Dataset configuration file should be a YAML file: {data_path}")
    return True


def setup_logging(loggers: list[str] | None):
    """
    Setup logging backends if specified.
    
    Following Ultralytics YOLO documentation:
    - Comet: Requires comet_ml.init() and API key in environment
    - ClearML: Requires clearml.browser_login() for authentication
    - TensorBoard: Automatically enabled, no initialization needed
    """
    if not loggers:
        return
    
    if "comet" in loggers:
        try:
            import comet_ml
            comet_ml.init()
            print("Comet logging initialized")
            print("Note: Ensure COMET_API_KEY is set in environment variables")
        except ImportError:
            print("Warning: comet_ml not installed. Install with: pip install comet_ml")
        except Exception as e:
            print(f"Warning: Failed to initialize Comet logging: {e}")
    
    if "clearml" in loggers:
        try:
            import clearml
            clearml.browser_login()
            print("ClearML logging initialized")
        except ImportError:
            print("Warning: clearml not installed. Install with: pip install clearml")
        except Exception as e:
            print(f"Warning: Failed to initialize ClearML logging: {e}")
    
    if "tensorboard" in loggers:
        print("TensorBoard logging enabled. View with: tensorboard --logdir runs/train")


def main():
    """Main training function."""
    args = parse_args()
    
    # Handle Roboflow dataset download
    dataset_yaml = args.data
    if args.roboflow:
        print("="*50)
        print("Downloading dataset from Roboflow...")
        print("="*50)
        
        downloaded_yaml = download_roboflow_dataset(
            url=args.roboflow_url,
            workspace=args.roboflow_workspace,
            project=args.roboflow_project,
            version=args.roboflow_version,
            api_key=args.roboflow_api_key
        )
        
        if downloaded_yaml is None:
            print("Failed to download Roboflow dataset")
            return
        
        dataset_yaml = downloaded_yaml
        print(f"Using dataset: {dataset_yaml}")
    
    # Validate dataset configuration
    if dataset_yaml is None:
        print("Error: Dataset configuration required. Use --data or --roboflow")
        return
    
    if not validate_dataset_config(dataset_yaml):
        return
    
    # Setup logging
    if args.loggers:
        setup_logging(args.loggers)
    
    # Load model
    print(f"Loading model: {args.model}")
    try:
        model = YOLO(args.model)
        
        # Load pretrained weights if specified and model is from YAML
        if args.pretrained and args.model.endswith(".yaml"):
            print(f"Loading pretrained weights: {args.pretrained}")
            model = YOLO(args.model).load(args.pretrained)
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    
    # Prepare training arguments
    train_kwargs = {
        "data": dataset_yaml,
        "epochs": args.epochs,
        "batch": args.batch,
        "imgsz": args.imgsz,
        "lr0": args.lr0,
        "lrf": args.lrf,
        "project": args.project,
        "save": args.save,
        "save_period": args.save_period,
    }
    
    # Add optional arguments
    if args.device is not None:
        # Handle device specification (can be string like "0,1" or list)
        if "," in args.device:
            train_kwargs["device"] = [int(d.strip()) for d in args.device.split(",")]
        elif args.device == "-1":
            train_kwargs["device"] = -1
        elif args.device.isdigit():
            train_kwargs["device"] = int(args.device)
        else:
            train_kwargs["device"] = args.device
    
    if args.name:
        train_kwargs["name"] = args.name
    
    if args.resume:
        train_kwargs["resume"] = True
    
    # Print training configuration
    print("\n" + "="*50)
    print("Training Configuration:")
    print("="*50)
    for key, value in train_kwargs.items():
        print(f"  {key}: {value}")
    print("="*50 + "\n")
    
    # Start training
    try:
        print("Starting training...")
        print("Training metrics will be logged automatically if loggers are configured.")
        results = model.train(**train_kwargs)
        
        print("\n" + "="*50)
        print("Training completed successfully!")
        print("="*50)
        
        # Get save directory from results or construct from project/name
        if results is not None and hasattr(results, 'save_dir'):
            save_dir = Path(results.save_dir)
        else:
            # Construct save directory from project and name
            project_dir = Path(train_kwargs.get("project", "runs/train"))
            if args.name:
                save_dir = project_dir / args.name
            else:
                # Find the most recent training directory
                if project_dir.exists():
                    dirs = sorted(project_dir.glob("*"), key=lambda p: p.stat().st_mtime, reverse=True)
                    if dirs:
                        save_dir = dirs[0]
                    else:
                        save_dir = project_dir / "exp"
                else:
                    save_dir = project_dir / "exp"
        
        print(f"Results saved to: {save_dir}")
        best_model = save_dir / "weights" / "best.pt"
        last_model = save_dir / "weights" / "last.pt"
        
        if best_model.exists():
            print(f"Best model: {best_model}")
        if last_model.exists():
            print(f"Last model: {last_model}")
        
        print("\nUse the best.pt model for ball position tracking in your application.")
        print("To export to ONNX format for ball_detect node:")
        print(f"  python -c \"from ultralytics import YOLO; YOLO('{best_model}').export(format='onnx')\"")
        
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
        print("You can resume training with --resume flag")
    except Exception as e:
        print(f"\nError during training: {e}")
        import traceback
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()
