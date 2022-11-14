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
    "left_upper_leg": dout.left_upper_leg.tolist(),
    "left_lower_leg": dout.left_lower_leg.tolist(),
    "right_upper_leg": dout.right_upper_leg.tolist(),
    "right_lower_leg": dout.right_lower_leg.tolist(),
    "left_upper_arm": dout.left_upper_arm.tolist(),
    "left_lower_arm": dout.left_lower_arm.tolist(),
    "left_hand": dout.left_hand.tolist(),
    "right_upper_arm": dout.right_upper_arm.tolist(),
    "right_lower_arm": dout.right_lower_arm.tolist(),
    "right_hand": dout.right_hand.tolist(),
    "waist": dout.waist.tolist(),
    "chest": dout.chest.tolist(),
    "head": dout.head.tolist()
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