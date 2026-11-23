"""Installed capabilities, with native NLP plugin probes isolated from clients."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from .conic_backends import CONIC_BACKENDS,conic_available
from .optimization import NLP_BACKENDS,EXPERIMENTAL_NLP_BACKENDS


def runtime_capabilities(crisp_binary=None):
    result=dict(nlp={},conic={name:conic_available(name) for name in CONIC_BACKENDS})
    for backend in NLP_BACKENDS+EXPERIMENTAL_NLP_BACKENDS:
        source='import casadi,json; print(json.dumps(bool(casadi.has_nlpsol('+repr(backend)+'))))'
        try:
            probe=subprocess.run([sys.executable,'-c',source],capture_output=True,text=True,timeout=20,
                                 env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1'))
            available=probe.returncode==0 and probe.stdout.strip().splitlines()[-1]=='true'
            detail=dict(available=available,experimental=backend in EXPERIMENTAL_NLP_BACKENDS,
                        enabled_by_default=backend=='ipopt',probe_exit_code=probe.returncode)
        except (subprocess.TimeoutExpired,IndexError):detail=dict(available=False,probe_failed=True)
        result['nlp'][backend]=detail
    binary=crisp_binary or os.environ.get('CONTACT_TRANSFER_CRISP_BINARY') or shutil.which('contact-transfer-crisp')
    result['simulation']=dict(
        mujoco=dict(installed=importlib.util.find_spec('mujoco') is not None,default_for_full_tasks=True),
        crisp=dict(bridge_implemented=True,binary_found=bool(binary and Path(binary).is_file() and os.access(binary,os.X_OK)),
                   binary=str(binary) if binary else None,sdk_bundled=False,
                   supported_scope='fixed-base torque Panda arms and one rigid panel; excludes native Robotiq closed loops',
                   full_tasks_qualified=False))
    return result
