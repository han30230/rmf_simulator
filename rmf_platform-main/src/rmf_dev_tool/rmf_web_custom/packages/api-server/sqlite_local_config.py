"""Local RMF API server configuration for the simulation workspace."""

from os.path import dirname

from api_server.default_config import config


here = dirname(__file__)
run_dir = f"{here}/run"

config.update(
    {
        "db_url": f"sqlite://{run_dir}/db.sqlite3",
        "cache_directory": f"{run_dir}/cache",
        "ros_args": ["-p", "use_sim_time:=true"],
        "log_level": "INFO",
        "host": "0.0.0.0",
        "port": 8100,
        "timezone": "Asia/Seoul",
    }
)
