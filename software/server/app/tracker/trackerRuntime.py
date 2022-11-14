import os, sys
import json
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from flask import Blueprint, current_app, session
from flask import redirect, url_for, request
from flask import jsonify, send_file
import random
from algorithm import Sensor_data_in

bp = Blueprint("tracker-runtime", __name__, url_prefix="/tracker-runtime")

@bp.route("/GyroAcc1", methods=["POST"])
def unity_tracking():
    #print("received")
    d = request.data.decode().split(",")
    #print(d[0], d[1], d[2], d[3], d[4], d[5], d[6])
    Sensor_data_in([d[0]], [d[1], d[2], d[3]], [d[4], d[5], d[6]])
    return {"retval":1 }