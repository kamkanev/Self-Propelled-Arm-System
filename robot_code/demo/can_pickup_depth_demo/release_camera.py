import gc
import time


def release_jetbot_camera():
    """Best-effort cleanup for JetBot's singleton camera object."""
    try:
        from jetbot import Camera
    except Exception as exc:
        print("[camera] could not import jetbot.Camera:", exc)
        return

    camera = None
    try:
        camera = Camera.instance()
        print("[camera] got Camera.instance()")
    except Exception as exc:
        print("[camera] Camera.instance() not available or could not start:", exc)

    if camera is not None:
        try:
            camera.stop()
            print("[camera] camera.stop() called")
        except Exception as exc:
            print("[camera] camera.stop() failed:", exc)

    # Some JetBot builds keep a singleton reference on the class. Clear common names
    # without assuming a specific version.
    for attr in ("_instance", "instance_obj", "camera"):
        if hasattr(Camera, attr):
            try:
                setattr(Camera, attr, None)
                print("[camera] cleared Camera.{}".format(attr))
            except Exception as exc:
                print("[camera] could not clear Camera.{}: {}".format(attr, exc))


def release_opencv():
    try:
        import cv2
    except Exception as exc:
        print("[opencv] could not import cv2:", exc)
        return

    try:
        cv2.destroyAllWindows()
        print("[opencv] destroyAllWindows() called")
    except Exception as exc:
        print("[opencv] destroyAllWindows() failed:", exc)


def main():
    print("[release] camera cleanup start")
    release_jetbot_camera()
    release_opencv()
    gc.collect()
    time.sleep(1.0)
    print("[release] camera cleanup done")


if __name__ == "__main__":
    main()
