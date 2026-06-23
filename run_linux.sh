#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${SCRIPT_DIR}/.venv/bin/python"

if [[ "$(uname -s)" == "Linux" ]] && ! id -nG | tr ' ' '\n' | grep -qx "dialout"; then
    if getent group dialout | grep -Eq "(^|,)${USER}(,|$)" && [[ "${PAROL6_DIALOUT_REEXEC:-0}" != "1" ]]; then
        export PAROL6_DIALOUT_REEXEC=1
        exec sg dialout -c "$(printf '%q ' "$0" "$@")"
    fi
fi

if [[ ! -x "${PYTHON_BIN}" ]]; then
    echo "Missing virtual environment Python: ${PYTHON_BIN}" >&2
    echo "Create it first with Python 3.10:" >&2
    echo "  cd ${SCRIPT_DIR}" >&2
    echo "  python3.10 -m venv .venv" >&2
    echo "  source .venv/bin/activate" >&2
    echo "  python -m pip install --upgrade pip setuptools" >&2
    echo "  python -m pip install wheel==0.42.0" >&2
    echo "  pip install -r requirements.txt" >&2
    exit 1
fi

cd "${SCRIPT_DIR}/GUI/files"
exec "${PYTHON_BIN}" Serial_sender_good_latest.py "$@"
