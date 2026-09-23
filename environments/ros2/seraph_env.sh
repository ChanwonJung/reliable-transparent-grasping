#!/usr/bin/env bash
# Robot runtime environment + Seraph SSH tunnels. SOURCE it (bash or zsh):
#
#     source environments/ros2/seraph_env.sh               # env + tunnels
#     source environments/ros2/seraph_env.sh --no-tunnel   # env only (off-network)
#
# Sets up:
#   1. ROS 2 Jazzy, the Kinova underlay, and this repo's ros2_ws overlay.
#   2. This repo's .venv site-packages on PYTHONPATH (requirements/ros2_runtime.txt).
#      Nodes run under the system python3 (colcon rewrites their shebangs), so
#      the venv is reached through PYTHONPATH rather than by activating it.
#   3. Loopback tunnels to the model servers on Seraph (aurora-g6), through the
#      aurora.khu.ac.kr jump host. These ports are also the client defaults in
#      the ROS packages — change both together.
#
#        10000  Qwen3.5-27B (SGLang, OpenAI-compatible HTTP)   qwen_server
#        20000  SAM 2.1 (ZMQ REQ/REP)                          sam_server
#        30000  SwinDRNet (ZMQ REQ/REP)                        graspgen_pkg
#        40000  GraspGen (ZMQ REQ/REP)                         graspgen_pkg
#
# Override the SSH endpoints without editing this file:
#     SERAPH_JUMP=user@aurora.khu.ac.kr:30080 SERAPH_HOST=user@aurora-g6 source ...

# ── Repo root (bash: BASH_SOURCE, zsh: %x) ──────────────────────────────────
if [ -n "${BASH_SOURCE[0]:-}" ]; then
    _RTG_SELF="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION:-}" ]; then
    _RTG_SELF="${(%):-%x}"
else
    _RTG_SELF="$0"
fi
RTG_ROOT="$(cd "$(dirname "${_RTG_SELF}")/../.." && pwd)"
export RTG_ROOT
unset _RTG_SELF

# ── ROS 2 ───────────────────────────────────────────────────────────────────
# ROS setup scripts pick their own shell flavour from the running shell.
if [ -n "${ZSH_VERSION:-}" ]; then _RTG_SH=zsh; else _RTG_SH=bash; fi
source "/opt/ros/jazzy/setup.${_RTG_SH}"
source "${HOME}/workspace/ros2_kortex_ws/install/setup.${_RTG_SH}"
if [ -f "${RTG_ROOT}/ros2_ws/install/setup.${_RTG_SH}" ]; then
    source "${RTG_ROOT}/ros2_ws/install/setup.${_RTG_SH}"
else
    echo "[seraph_env] ros2_ws not built yet — run colcon build in ${RTG_ROOT}/ros2_ws"
fi
unset _RTG_SH

# ── Python deps (.venv) ─────────────────────────────────────────────────────
# Must match the system python3 the nodes run under. No glob: zsh errors on
# a pattern with no match.
_RTG_SITE="${RTG_ROOT}/.venv/lib/python$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')/site-packages"
if [ -d "${_RTG_SITE}" ]; then
    case ":${PYTHONPATH:-}:" in
        *":${_RTG_SITE}:"*) ;;
        *) export PYTHONPATH="${_RTG_SITE}${PYTHONPATH:+:${PYTHONPATH}}" ;;
    esac
    echo "[seraph_env] venv: ${_RTG_SITE}"
else
    echo "[seraph_env] ✗ no .venv — the ZMQ / Qwen clients will fail. Create it:"
    echo "    python3 -m venv --system-site-packages ${RTG_ROOT}/.venv"
    echo "    ${RTG_ROOT}/.venv/bin/pip install -r ${RTG_ROOT}/requirements/ros2_runtime.txt"
fi
unset _RTG_SITE

# ── SSH tunnels ─────────────────────────────────────────────────────────────
_RTG_SKIP_TUNNEL="${SKIP_SERAPH_TUNNEL:-0}"
for _rtg_arg in "$@"; do
    [ "${_rtg_arg}" = "--no-tunnel" ] && _RTG_SKIP_TUNNEL=1
done
unset _rtg_arg

SERAPH_JUMP="${SERAPH_JUMP:-allen516@aurora.khu.ac.kr:30080}"
SERAPH_HOST="${SERAPH_HOST:-allen516@aurora-g6}"

# A LISTENER on the port — a bare `lsof -i` also matches outbound sockets left
# in CLOSE_WAIT, which would make a dead tunnel look open.
_rtg_port_listening() {
    ss -Htln "sport = :$1" 2>/dev/null | grep -q .
}

# `ssh -f` backgrounds itself and exits 0 even when the forward fails to bind,
# so poll for the listener instead of trusting the exit code.
_rtg_open_tunnel() {
    local port="$1" label="$2" i
    if _rtg_port_listening "${port}"; then
        echo "[seraph_env] ✓ ${label} tunnel already open (localhost:${port})"
        return 0
    fi
    if ! ssh -fN -o ConnectTimeout=10 -o ExitOnForwardFailure=yes \
            -L "${port}:127.0.0.1:${port}" -J "${SERAPH_JUMP}" "${SERAPH_HOST}"; then
        echo "[seraph_env] ✗ ${label} tunnel FAILED (port ${port})"
        return 1
    fi
    for i in 1 2 3 4 5 6; do
        if _rtg_port_listening "${port}"; then
            echo "[seraph_env] ✓ ${label} tunnel → localhost:${port}"
            return 0
        fi
        sleep 0.5
    done
    echo "[seraph_env] ✗ ${label}: ssh returned 0 but nothing listens on ${port}"
    return 1
}

if [ "${_RTG_SKIP_TUNNEL}" = "1" ]; then
    echo "[seraph_env] tunnels skipped (--no-tunnel / SKIP_SERAPH_TUNNEL=1)"
else
    _rtg_fails=0
    _rtg_open_tunnel 10000 "Qwen (SGLang)" || _rtg_fails=$((_rtg_fails + 1))
    _rtg_open_tunnel 20000 "SAM 2.1"       || _rtg_fails=$((_rtg_fails + 1))
    _rtg_open_tunnel 30000 "SwinDRNet"     || _rtg_fails=$((_rtg_fails + 1))
    _rtg_open_tunnel 40000 "GraspGen"      || _rtg_fails=$((_rtg_fails + 1))
    if [ "${_rtg_fails}" -ne 0 ]; then
        echo "[seraph_env] ⚠ ${_rtg_fails} tunnel(s) down — those clients will fail to connect."
    fi
    unset _rtg_fails
fi
unset _RTG_SKIP_TUNNEL
unset -f _rtg_open_tunnel _rtg_port_listening
