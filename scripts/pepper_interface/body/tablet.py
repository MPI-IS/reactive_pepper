class Tablet :

    def __init__(self,tablet_proxy):
        self._tablet = tablet_proxy

    def set_background(self,color):
        self._tablet.setBackgroundColor(color)

    def set_image(self,url):
        self._tablet.showImage(url)

    def load_image(self,url):
        return self._table.preLoadImage(url)
