class Music:

    def __init__(self,music_proxy):
        self._music = music_proxy
        self._song_id = None

    def load_music(self, music):
        self._song_id = self._music.loadFile(music)

    def unload_music(self):
        self._music.unloadFile(self._song_id)

    def play_music(self):

        self._music.post.playInLoop(self._song_id)

    def stop_music(self):
        self._music.stop(self._song_id)

    def stop_all_music(self):
        self._music.stopAll()
