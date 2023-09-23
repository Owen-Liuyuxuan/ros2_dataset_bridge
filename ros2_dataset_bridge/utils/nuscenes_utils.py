"""

"""
import threading
import time

_NUSC_GLOBAL_DICT = {}
def NuScenes(dataroot, version, *args,  **kwargs):
    if (dataroot, version) not in _NUSC_GLOBAL_DICT:
        from nuscenes.nuscenes import NuScenes as NuSceneObj
        _NUSC_GLOBAL_DICT[(dataroot, version)] = NuSceneObj(version=version, dataroot=dataroot, *args, **kwargs)
    return _NUSC_GLOBAL_DICT[(dataroot, version)]

class NuscenesLoader():
    def __init__(self, dataroot, version, *args, **kwargs):
        self._nusc = None
        self.lock = threading.Lock()
        self.t = threading.Thread(target=self.preload, args=[dataroot, version] + list(args), kwargs=kwargs)
        self.t.start()

    def get_nusc(self, logger, max_timeout=600):
        start_time = time.time()
        while True:
            if time.time() - start_time > max_timeout:
                raise TimeoutError("nuscenes not loaded in time")
            if self.lock.locked():
                logger.info("Waiting for nuscenes to load")
                time.sleep(0.5)
            else:
                logger.info("nuscenes loaded")
                return self._nusc

    def preload(self, dataroot, version, *args, **kwargs):
        with self.lock:
            self._nusc = NuScenes(dataroot, version, *args, **kwargs)

