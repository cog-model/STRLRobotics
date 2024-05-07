import pickle
import codecs
from typing import Any


def str2obj(raw_data: str):
    if raw_data is None:
        return None
    object = pickle.loads(codecs.decode(raw_data.encode("latin1"), "base64"))
    return object


def obj2str(obj: Any):
    if obj is None:
        return None
    raw_data = codecs.encode(
        pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL), "base64"
    ).decode("latin1")
    return raw_data
