"""The A3 Ultra T3D0 contract (``robots/a3_ultra.py``, ``sonic_a3_ultra_t3d0.yaml``) keeps the Litchi A3 (a3_024) low-level contract.

``--robot a3_ultra`` itself now trains the A3 contract (``sonic_a3_ultra.yaml``, see ``test_a3_ultra_training_recipe.py``), so
its Kp/Kd are the A3's by construction.  The Ultra's own 63-DOF T3D0 contract is still available (``+exp=...sonic_a3_ultra_t3d0``)
and must stay on the same numbers.  Three places hold them and must not drift apart:

* ``robots/a3.py``        -- Litchi's ``A3_024_CYLINDER_CFG`` (what the released 035 checkpoint was trained with),
* ``robots/a3_ultra.py``  -- ``A3_ULTRA_CFG``, built from the above while ``A3_ULTRA_GAIN_SOURCE == "litchi_a3_024"``,
* ``gear_sonic_deploy/.../a3_policy_parameters.hpp`` -- ``a3_024_kps`` / ``a3_024_kds`` / ``a3_024_action_scale`` /
  ``a3_default_angles`` used by ``a3_deploy_onnx_ref`` for both ``--robot a3`` and ``--robot a3_ultra``.

The robot modules need Isaac Sim's ``pxr`` to import normally, so they are loaded in a subprocess with the isaaclab
config classes replaced by plain attribute containers (we only read numbers).  A subprocess keeps those stubs out of
the rest of the test session.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import textwrap

import pytest

REPO = Path(__file__).resolve().parents[1]
HEADER = REPO / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_policy_parameters.hpp"

_LOADER = textwrap.dedent(
    r'''
    import json, re, sys, types
    sys.path.insert(0, REPO)

    class _Meta(type):
        def __getattr__(cls, name):
            if name.startswith("__"):
                raise AttributeError(name)
            return _Cfg

    class _Cfg(metaclass=_Meta):
        def __init__(self, *a, **kw):
            self.__dict__.update(kw)
        def __getattr__(self, name):
            if name.startswith("__"):
                raise AttributeError(name)
            return _Cfg
        def replace(self, **kw):
            import copy
            c = copy.deepcopy(self); c.__dict__.update(kw); return c

    class _Mod(types.ModuleType):
        def __getattr__(self, name):
            if name.startswith("__"):
                raise AttributeError(name)
            return _Cfg

    for name in ["isaaclab", "isaaclab.actuators", "isaaclab.assets", "isaaclab.assets.articulation", "isaaclab.sim",
                 "isaaclab.utils", "isaaclab.managers", "isaaclab.terrains", "isaaclab.envs", "isaaclab.sensors"]:
        m = sys.modules.setdefault(name, _Mod(name))
        if "." in name:
            parent, child = name.rsplit(".", 1)
            setattr(sys.modules[parent], child, m)

    from gear_sonic.envs.manager_env.robots import a3, a3_ultra

    KEYS = ("stiffness", "damping", "effort_limit_sim", "velocity_limit_sim", "armature", "friction", "dynamic_friction")

    def table(cfg):
        rows = []
        for group, act in cfg.actuators.items():
            for expr in act.joint_names_expr:
                row = {"group": group, "expr": expr}
                for k in KEYS:
                    v = act.__dict__.get(k)
                    row[k] = v.get(expr) if isinstance(v, dict) else v
                rows.append(row)
        return rows

    def resolve(rows, joint):
        for r in rows:
            if re.fullmatch(r["expr"], joint):
                return {k: r[k] for k in KEYS}
        return None

    def scale_for(scale, joint):
        for expr, v in scale.items():
            if re.fullmatch(expr, joint):
                return v
        return None

    body = list(a3.A3_ACTIVE_JOINT_NAMES)
    head = list(a3_ultra.A3_ULTRA_HEAD_JOINTS)
    A, U = table(a3.A3_024_CYLINDER_CFG), table(a3_ultra.A3_ULTRA_CFG)
    out = {
        "source": a3_ultra.A3_ULTRA_GAIN_SOURCE,
        "body": body,
        "head": head,
        "a3": {j: resolve(A, j) for j in body},
        "a3u": {j: resolve(U, j) for j in body + head},
        "a3_scale": {j: scale_for(a3.A3_024_ACTION_SCALE, j) for j in body},
        "a3u_scale": {j: scale_for(a3_ultra.A3_ULTRA_ACTION_SCALE, j) for j in body + head},
        "a3_init": {j: a3.A3_024_CYLINDER_CFG.init_state.joint_pos.get(j) for j in body},
        "a3u_init": {j: a3_ultra.A3_ULTRA_CFG.init_state.joint_pos.get(j) for j in body + head},
        "a3_pos": list(a3.A3_024_CYLINDER_CFG.init_state.pos),
        "a3u_pos": list(a3_ultra.A3_ULTRA_CFG.init_state.pos),
        "a3u_groups": sorted(a3_ultra.A3_ULTRA_CFG.actuators),
        "head_kp_kd": [a3_ultra.A3_ULTRA_HEAD_KP, a3_ultra.A3_ULTRA_HEAD_KD],
    }
    # the vendor-doc path must keep working (it is the documented way back to the official numbers)
    vend = a3_ultra._vendor_doc_actuators()
    out["vendor_hip"] = [vend["hip"].stiffness, vend["hip"].damping]
    out["vendor_init_z"] = a3_ultra._vendor_doc_init_state().pos[2]
    print("JSON:" + json.dumps(out))
    '''
)


@pytest.fixture(scope="module")
def contract() -> dict:
    code = f"REPO = {str(REPO)!r}\n" + _LOADER
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, proc.stderr[-2000:]
    line = next(l for l in proc.stdout.splitlines() if l.startswith("JSON:"))
    return json.loads(line[len("JSON:") :])


def test_active_source_is_the_litchi_a3_contract(contract):
    assert contract["source"] == "litchi_a3_024"


@pytest.mark.parametrize(
    "key", ["stiffness", "damping", "effort_limit_sim", "velocity_limit_sim", "armature", "friction", "dynamic_friction"]
)
def test_body_joint_parameters_equal_litchi_a3(contract, key):
    for joint in contract["body"]:
        a, u = contract["a3"][joint], contract["a3u"][joint]
        assert a is not None and u is not None, joint
        assert u[key] == pytest.approx(a[key]), f"{joint}.{key}: A3 Ultra {u[key]} != Litchi A3_024 {a[key]}"


def test_action_scale_default_pose_and_spawn_equal_litchi_a3(contract):
    for joint in contract["body"]:
        assert contract["a3u_scale"][joint] == pytest.approx(contract["a3_scale"][joint]), joint
        assert contract["a3u_init"][joint] == pytest.approx(contract["a3_init"][joint]), joint
    assert contract["a3u_pos"] == pytest.approx(contract["a3_pos"])


def test_head_joints_use_the_deploy_code_fixed_head_gains(contract):
    assert contract["head_kp_kd"] == [40.0, 2.0]  # kA3HeadKp / kA3HeadKd in expand_to_backend.hpp
    expand = (REPO / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_deploy/expand_to_backend.hpp").read_text()
    assert re.search(r"kA3HeadKp\s*=\s*40\.0", expand) and re.search(r"kA3HeadKd\s*=\s*2\.0", expand)
    for joint in contract["head"]:
        u = contract["a3u"][joint]
        assert (u["stiffness"], u["damping"]) == (40.0, 2.0)
        assert contract["a3u_init"][joint] == 0.0  # deploy holds the head at q = 0


def test_passive_foot_springs_are_not_carried_over(contract):
    assert contract["a3u_groups"] == ["arms", "feet", "head", "legs", "waist"]


def test_vendor_doc_values_are_preserved_for_a_later_switch(contract):
    assert contract["vendor_hip"] == [200.0, 7.0]
    assert contract["vendor_init_z"] == pytest.approx(1.2)


@pytest.mark.skipif(shutil.which("g++") is None, reason="needs a C++ compiler to read the deploy header")
def test_deploy_header_matches_training_contract(contract, tmp_path):
    """The numbers a3_deploy_onnx_ref sends (profile a3_024) equal what a3_ultra trains with."""
    src = HEADER.read_text()
    block = src[src.index("a3_default_angles = {") :]
    block = block[: block.index("};")]
    names = [m.group(2) for m in re.finditer(r"//\s*\[\s*(\d+)\]\s*(\w+)", block)]
    assert len(names) == 29

    cpp = tmp_path / "dump.cpp"
    cpp.write_text(
        '#include <cstdio>\n#include "a3_policy_parameters.hpp"\n'
        "static void dump(const char* n, const std::array<double,29>& a){std::printf(\"%s\", n);"
        'for (double v : a) std::printf(" %.17g", v); std::printf("\\n");}\n'
        'int main(){dump("default", a3_default_angles); dump("kp", a3_024_kps); dump("kd", a3_024_kds);'
        'dump("scale", a3_024_action_scale);}\n'
    )
    exe = tmp_path / "dump"
    build = subprocess.run(
        ["g++", "-std=c++20", "-I", str(HEADER.parent), str(cpp), "-o", str(exe)], capture_output=True, text=True
    )
    assert build.returncode == 0, build.stderr[-1500:]
    rows = {}
    for line in subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.splitlines():
        key, *vals = line.split()
        rows[key] = dict(zip(names, map(float, vals)))

    for joint in contract["body"]:
        u = contract["a3u"][joint]
        assert rows["kp"][joint] == pytest.approx(u["stiffness"]), f"{joint} kp: deploy vs a3_ultra training"
        assert rows["kd"][joint] == pytest.approx(u["damping"]), f"{joint} kd: deploy vs a3_ultra training"
        assert rows["scale"][joint] == pytest.approx(contract["a3u_scale"][joint]), f"{joint} action scale"
        assert rows["default"][joint] == pytest.approx(contract["a3u_init"][joint]), f"{joint} default angle"


@pytest.mark.parametrize(
    "mjcf",
    [
        "gear_sonic/data/robot_model/model_data/a3_ultra/a3_ultra_t3d0.xml",  # MuJoCo simulator (run_sim_loop.py)
        "gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml",  # motion-library FK
    ],
)
def test_mjcf_armature_matches_the_training_contract(contract, mjcf):
    """The simulator's rotor inertia must be the armature the policy contract trains with (wrist_roll once wasn't)."""
    mujoco = pytest.importorskip("mujoco")
    path = REPO / mjcf
    if not path.exists():
        pytest.skip(f"{mjcf} not present")
    model = mujoco.MjModel.from_xml_path(str(path))
    for joint in contract["body"] + contract["head"]:
        dof = model.jnt_dofadr[model.joint(joint).id]
        assert model.dof_armature[dof] == pytest.approx(contract["a3u"][joint]["armature"], rel=1e-3), joint


def test_a3_sim_yaml_armature_matches_the_contract(contract):
    """`--robot a3` simulates the T2D5 MJCF, which has armature 0; the sim fills in the contract's rotor inertia."""
    yaml = pytest.importorskip("yaml")
    cfg = yaml.safe_load((REPO / "gear_sonic/utils/mujoco_sim/wbc_configs/a3_035.yaml").read_text())
    table = cfg["JOINT_ARMATURE_IF_MISSING"]
    assert sorted(table) == sorted(contract["body"])
    for joint in contract["body"]:
        assert table[joint] == pytest.approx(contract["a3"][joint]["armature"], rel=1e-3), joint
