"""Robot model instantiation helpers and the robot-name registry used by `--robot`."""

from .g1 import instantiate_g1_robot_model

# Add an entry here once a robot has both an instantiate_*_robot_model() function
# and (for RobotSupplementalInfo-based IK/teleop) a *SupplementalInfo class.
# See docs/source/user_guide/new_embodiments.md.
ROBOT_MODEL_REGISTRY = {
    "g1": instantiate_g1_robot_model,
}


def instantiate_robot_model(robot: str, **kwargs):
    """Instantiate a RobotModel for `robot` via the registry. Raises NotImplementedError
    for robots that don't have an instantiate_*_robot_model() registered yet."""
    if robot not in ROBOT_MODEL_REGISTRY:
        raise NotImplementedError(
            f"No instantiate_*_robot_model() registered for robot='{robot}'. "
            f"Available: {list(ROBOT_MODEL_REGISTRY)}. "
            "See docs/source/user_guide/new_embodiments.md to add one."
        )
    return ROBOT_MODEL_REGISTRY[robot](**kwargs)


__all__ = ["instantiate_g1_robot_model", "ROBOT_MODEL_REGISTRY", "instantiate_robot_model"]
