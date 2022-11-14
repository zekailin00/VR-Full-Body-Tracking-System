import os, sys

from flask import Blueprint
from flask import request
import json

from algorithm import VR_data_in
import algorithm.output_struct as dout

bp = Blueprint("unity-runtime", __name__, url_prefix="/unity-runtime")

@bp.route("/pose-data", methods=["GET"])
def unity_pose():

    return {
    "left_upper_leg": dout.left_lower_leg,
    "left_lower_leg": dout.left_lower_leg,
    "right_upper_leg": dout.right_upper_leg,
    "right_lower_leg": dout.right_lower_leg,
    "left_upper_arm": dout.left_upper_arm,
    "left_lower_arm": dout.left_lower_arm,
    "left_hand": dout.left_hand,
    "right_upper_arm": dout.right_upper_arm,
    "right_lower_arm": dout.right_lower_arm,
    "right_hand": dout.right_hand,
    "waist": dout.waist,
    "chest": dout.chest,
    "head": dout.head
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

    return {"retval":1 }