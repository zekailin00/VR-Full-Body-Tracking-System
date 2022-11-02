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
    print("received")
    data = request.data
   # dataDict = json.loads(data)
   # print(dataDict)
    print(data)
    Sensor_data_in(
        [data["HRX"], d["HRY"], d["HRZ"]], 
        [d["HPX"], d["HPY"], d["HPZ"]], 
        [d["LRX"], d["LRY"], d["LRZ"]], 
        [d["LPX"], d["LPY"], d["LPZ"]], 
        [d["RRX"], d["RRY"], d["RRZ"]], 
        [d["RPX"], d["RPY"], d["RPZ"]])

    return {"retval":1 }
