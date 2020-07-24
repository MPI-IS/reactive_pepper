
from .velocity_to_duration import velocity_to_duration
import threading
import time

class Dance_manager:

    def __init__(self,
                 motion_proxy,
                 postures,
                 beat,
                 initial_velocity,
                 batch_size=5):

        self._postures = postures
        self._beat = 1.0 / float(beat)
        self._initial_velocity = initial_velocity
        self._mid = None
        self._batch_size = batch_size
        self._motion = motion_proxy
        self._lock = threading.Lock()

        
    def start(self, current_values, next_beat):

        # motion from arbitrary position to first posture

        target_posture = self._postures[0]
        duration = velocity_to_duration(target_posture,
                                        current_values,
                                        self._initial_velocity)

        joints = target_posture.keys()
        values = [target_posture[joint] for joint in joints]

        # we align the arrival to first posture with beat
        duration = self.align_duration(duration, next_beat)

        # moving to first posture
        self._mid = self._motion.post.angleInterpolation(
            joints, values, duration, True)

        
    def overwrite_postures(self,new_postures):

        with self._lock:
            self._postures = new_postures

            
    def align_duration(self, duration, next_beat):

        t = time.time()

        # oops, this comes too late
        while t > next_beat:
            next_beat = next_beat + self._beat

        # we slow down a bit to match the beat
        if (t + duration) < next_beat:
            return next_beat - t

        # we accelerate reasonably to match the beat
        diff = t + duration - next_beat
        if diff < 0.2:
            return duration - diff

        # we slow down a lot to get the following beat
        return (next_beat + self._beat) - t

    
    def create_command(self, next_beat):

        joints = self._postures[0].keys()
        values = []
        durations = []

        with self._lock:
            for joint in joints:
                values_ = []
                for _ in range(self._batch_size):
                    for position in self._postures:
                        values_.append(position[joint])
                values.append(values_)

        nb_beats = len(self._postures) * self._batch_size

        first_duration = self.align_duration(self._beat, next_beat)

        for _ in range(len(joints)):
            t = first_duration
            durations_ = []
            for _ in range(nb_beats):
                durations_.append(t)
                t += self._beat
            durations.append(durations_)

        return joints, values, durations

    
    def update(self, next_beat):
        if not self._motion.isRunning(self._mid):
            joints, values, durations = self.create_command(next_beat)
            self._mid = self._motion.post.angleInterpolation(joints,
                                                             values,
                                                             durations,
                                                             True)

            
    def stop(self):

        try:
            self._motion.stop(self._mid)
        except:
            pass
