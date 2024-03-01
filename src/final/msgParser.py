import json


def parseLaneMsg(data):
    data = json.loads(data)
    return data


def dumpLaneMsg(state, left_fit, right_fit, img_height):
    data = {
        "lane": state,
        "left_fit": left_fit,
        "right_fit": right_fit,
        "img_height": img_height,
    }
    return json.dumps(data)
