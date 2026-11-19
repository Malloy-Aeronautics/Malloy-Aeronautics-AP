#!/usr/bin/env python3
'''
The MA airframes SITL flies: each is a SITL frame, a physics model (models/<model>.json) and the
parameter files loaded on top of default_params/copter.parm. The empty and payload versions of an
aircraft share its parameter files and differ only in the model's mass and hover throttle.

Used by the autotests (MA_AIRFRAME=<name>) and by Tools/scripts/ma_airframe_tests.sh, which flies
every airframe here that has its model.

  python3 Tools/autotest/ma_airframes.py            the airframes and whether each one can fly
  python3 Tools/autotest/ma_airframes.py --ready    only the names of the ones that can fly
'''

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, "models")

# name: (SITL frame, model, parameter files in models/, in load order)
AIRFRAMES = {
    "T150_empty":   ("octa-quad", "T150_empty_sim",   ["T150", "T150_sitl"]),
    "T150_payload": ("octa-quad", "T150_payload_sim", ["T150", "T150_sitl"]),
    "T400_empty":   ("octa-quad", "T400_empty_sim",   ["T400", "T400_sitl"]),
    "T400_payload": ("octa-quad", "T400_payload_sim", ["T400", "T400_sitl"]),
}


def _model_file(name):
    return os.path.join(MODELS, AIRFRAMES[name][1] + ".json")


def ready(name):
    '''True if the airframe has its model and parameter files'''
    frame, model, params = AIRFRAMES[name]
    return os.path.exists(_model_file(name)) and all(
        os.path.exists(os.path.join(MODELS, p + ".param")) for p in params)


def defaults(name):
    '''the --defaults files for SITL, in load order'''
    return [os.path.join(HERE, "default_params", "copter.parm")] + [
        os.path.join(MODELS, p + ".param") for p in AIRFRAMES[name][2]]


def model(name):
    '''the --model argument for SITL'''
    frame, model, params = AIRFRAMES[name]
    return "%s:@ROMFS/models/%s.json" % (frame, model)


def model_data(name):
    with open(_model_file(name)) as f:
        return json.load(f)


def main():
    names = [n for n in AIRFRAMES if ready(n)]
    if "--ready" in sys.argv:
        print(" ".join(names))
        return
    for n in AIRFRAMES:
        if ready(n):
            m = model_data(n)
            print("%-14s ready    %5.0f kg, hover throttle %.2f" % (n, m["mass"], m["hoverThrOut"]))
        else:
            print("%-14s waiting  no %s yet" % (n, os.path.relpath(_model_file(n), os.path.dirname(HERE))))


if __name__ == "__main__":
    main()
