import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flask import Blueprint, current_app, session
from flask import redirect, url_for, request
from flask import jsonify, send_file
import random


bp = Blueprint("unity-runtime", __name__, url_prefix="/unity-runtime")

@bp.route("/pose-data", methods=["GET"])
def unity_pose():

    return {
    "obj1":1,
    "obj2":2
    }

@bp.route("/headset-data", methods=["POST"])
def unity_tracking():

    data = request.form['tracking']

    print(data)

    return {"retval":1 }