try:
    from ._hole_info import HoleInfo
except ImportError:
    try:
        from .hole_info import HoleInfo
    except ImportError:
        raise ImportError("Cannot import HoleInfo from cannyedge_test1.msg")


