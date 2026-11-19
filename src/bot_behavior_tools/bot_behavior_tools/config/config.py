from pathlib import Path
import os
import logging

# ===================================================
# Paths Configuration
# ===================================================

from ament_index_python.packages import get_package_share_directory

SIMULATOR_VERSION = os.environ.get("SIMULATOR_VERSION", "Gazebo Classic")
WORKSPACE_ROOT = Path(os.environ.get("BOT_WORKSPACE", Path.cwd())).resolve()
WORKSPACE_NAME = WORKSPACE_ROOT.name
PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_PACKAGE_DIR = WORKSPACE_ROOT / "src" / "bot_behavior_tools"
_installed = Path(get_package_share_directory("bot_behavior_tools"))
_candidate = WORKSPACE_ROOT / "src" / "bot_gazebo"
WRAPPER_PATH = _candidate if _candidate.is_dir() else Path(get_package_share_directory("bot_gazebo"))
PROMPTS_DIR = _installed / "prompts"
TEMPLATES_DIR = _installed / "templates"
WORLD_DIR = WRAPPER_PATH / "worlds"
BT_OUTPUT_DIR = Path(os.environ.get("BOT_BEHAVIOR_DIR", str(WRAPPER_PATH / "behavior_trees")))
SCENARIOS_DIR = WRAPPER_PATH / "scenarios"

# ===================================================
# LLM Configuration
# ===================================================
LLM_API_KEY = os.environ.get("BOT_LLM_API_KEY", "")
LLM_API_BASE_URL = os.environ.get("BOT_LLM_BASE_URL", "http://localhost:8000/v1")
LLM_MODEL = os.environ.get("BOT_LLM_MODEL", "")

# Previous model used :
# Qwen/Qwen2.5-VL-32B-Instruct-AWQ

LLM_TIMEOUT = 30  # seconds
LLM_RETRIES = 3
LLM_BACKOFF_FACTOR = 0.3

LLM_MAX_TOKENS = 4096
LLM_TEMPERATURE = 0.7

# ===================================================
# Logging Configuration
# ===================================================
LOG_DIR = Path(os.environ.get("BOT_RUNTIME_DIR", str(WORKSPACE_ROOT / ".runtime"))) / "behavior_logs"
LOG_FILE = LOG_DIR / "app.log"
LOG_FORMAT = '%(asctime)s - %(levelname)s - %(message)s'
LOG_DATE_FORMAT = '%m-%d %H:%M:%S'

# ===================================================
# Scenarios Configuration
# ===================================================
SUPPORTED_SCENARIO_EXTENSIONS = [".yaml", ".yml"]

def get_scenarios_directory():
    """ Get the scenarios directory path. """
    scenarios_path = Path(SCENARIOS_DIR).resolve()
    if not scenarios_path.exists():
        logging.warning(f"Scenarios directory not found: {scenarios_path}")
        scenarios_path = Path("scenarios").resolve()
    return str(scenarios_path)

# ===================================================
# Default Settings
# ===================================================
DEFAULT_BT_FILENAME = "generated_behavior_tree.xml"
DEFAULT_WORLD_FILE = WORLD_DIR / "house.world"

def setup_logging():
    """ Setup logging configuration. """
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        
        handlers = [logging.FileHandler(LOG_FILE)]
    except (PermissionError, OSError):
        handlers = [logging.StreamHandler()]
    
    logging.basicConfig(
        level=logging.INFO,
        format=LOG_FORMAT,
        datefmt=LOG_DATE_FORMAT,
        handlers=handlers,
        force=True
    )
    
    return logging.getLogger(__name__)