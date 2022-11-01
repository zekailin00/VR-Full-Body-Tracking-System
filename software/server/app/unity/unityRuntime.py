import os, sys

from flask import Blueprint
from flask import request
import json

from algorithm import VR_data_in


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
    d = json.loads(data)

    VR_data_in(
        [d["HRX"], d["HRY"], d["HRZ"]], 
        [d["HPX"], d["HPY"], d["HPZ"]], 
        [d["LRX"], d["LRY"], d["LRZ"]], 
        [d["LPX"], d["LPY"], d["LPZ"]], 
        [d["RRX"], d["RRY"], d["RRZ"]], 
        [d["RPX"], d["RPY"], d["RPZ"]])

    #print(dataDict)

    return {"retval":1 }