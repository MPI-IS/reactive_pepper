class TTS:

    def __init__(self,tts_proxy):
        self._tts = tts_proxy
        self._mid = None

    def set(self,text):
        new_mid = self._tts.post.say(text)
        try:
            self._tts.stop(self._mid)
        except:
            pass
        self._mid = new_mid

    def set_speech_speed(self, speed):

        self._tts.setParameter("speed",speed)

    def reset_speech_speed(self):

        self._tts.resetSpeed()
