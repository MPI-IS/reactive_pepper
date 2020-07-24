import time,math,copy

class _Event :

    FADE_BLUE = 1
    FADE_VIOLET = 2

    def __init__(self,
                 leds,
                 duration,
                 internal,
                 top,
                 external,
                 bottom,
                 chest,
                 ears,
                 brain_back,
                 brain_middle,
                 brain_front):
        
        self.leds = leds
        self.duration = duration

        self.PLAYFUL_VIOLET=[0.0,1.0,1.0,None]
        self.PLAYFUL_BLUE=[1.0,0.0,1.0,None]

        self.PLAYFUL_BLUE[3]=duration
        self.PLAYFUL_VIOLET[3]=duration

        self.d = { "FaceLedsBottom":None,
                  "FaceLedsExternal":None,
                  "FaceLedsTop":None,
                  "FaceLedsInternal":None }

        self.d["FaceLedsBottom"]=bottom
        self.d["FaceLedsExternal"]=external
        self.d["FaceLedsInternal"]=internal
        self.d["FaceLedsTop"]=top
        self.d["ChestLeds"]=chest

        self.ears = ears
        self.brain_back = brain_back
        self.brain_middle = brain_middle
        self.brain_front = brain_front

    def perform(self):

        for led,color in self.d.iteritems():
            if color:
                c = [c_ for c_ in color]
                c.append(self.duration)
                self.leds.post.fadeRGB(led,*c)


        if self.ears is not None:
            self.leds.post.fade("EarsLeds",self.ears,self.duration)

        if self.brain_back is not None:
            self.leds.post.fade("BrainLedsBack",self.brain_back,self.duration)

        if self.brain_middle is not None:
            self.leds.post.fade("BrainLedsMiddle",self.brain_middle,self.duration)

        if self.brain_front is not None:
            self.leds.post.fade("BrainLedsFront",self.brain_front,self.duration)

        time.sleep(self.duration)


def _blink_events(leds,color1,color2,duration,eyes=True,chest=True):

    blue_eyes = None
    violet_eyes = None

    blue_chest = None
    violet_chest = None

    if eyes:
        blue_eyes = color1
        violet_eyes = color2

    if chest :
        blue_chest = color1
        violet_chest = color2
    
    events = [ _Event(leds,duration,blue_eyes,blue_eyes,None,None,None,None,1,None,None),
               _Event(leds,duration,blue_eyes,blue_eyes,None,None,None,None,1,1,None),
               _Event(leds,duration,None,None,blue_eyes,blue_eyes,blue_chest,1,1,1,1),
               _Event(leds,duration,None,None,None,None,None,None,None,None,0),
               _Event(leds,duration,None,None,violet_eyes,violet_eyes,None,None,None,0,None),
               _Event(leds,duration,violet_eyes,violet_eyes,None,None,violet_chest,0,0,0,0),
               _Event(leds,duration,None,None,None,None,None,None,None,None,None) ]
    
    return events


def _blink(leds,
           color1,
           color2,
           eyes=True,
           chest=True,
           duration=0.01):

    events = _blink_events(leds,
                           color1,
                           color2,
                           duration,
                           eyes = eyes,
                           chest = chest)

    for e in events: e.perform()    

    time.sleep(duration*10)

    events = _blink_events(leds,
                           color1,
                           color2,
                           duration,
                           eyes=eyes,
                           chest=chest)

    for e in events: e.perform()    

    

class Leds:


    def __init__(self,leds_proxy):

        self._leds = leds_proxy

        self._colors = {}
        self._groups = {}
        
        self._colors["red"] = {"red": 1, "green": 0, "blue": 0}
    	self._colors["green"] = {"red": 0, "green": 1, "blue": 0}
    	self._colors["blue"] = {"red": 0, "green": 0, "blue": 1}
    	self._colors["purple"] = {"red": 0.5, "green": 0, "blue": 0.5}
    	self._colors["yellow"] = {"red": 1, "green": 1, "blue": 0}
      	self._colors["orange"] = {"red": 0.95, "green": 0.4, "blue": 0.0}
    	self._colors["playful"] = {"red": 0.0, "green": 1.0, "blue": 1.0}
        self._colors["playful_pink"] = {"red":1.0,"green":0.0, "blue":1.0}
        self._colors["pink"] = {"red":1.0,"green":0.71, "blue":0.78}
        
    	self._groups["ALL"] = {"red": ["AllLedsRed"],
                                   "green": ["AllLedsGreen"],
                                   "blue": ["AllLedsBlue"]}

    	self._groups["EYES"] = {"red": ["RightFaceLedsRed", "LeftFaceLedsRed"],
                                    "green": ["RightFaceLedsGreen", "LeftFaceLedsGreen"],
                                    "blue": ["RightFaceLedsBlue", "LeftFaceLedsBlue"]}

        self._groups["CHEST"] = {"red": ["ChestLedsRed"],
                                 "green": ["ChestLedsGreen"],
                                 "blue": ["ChestLedsBlue"]}

        groups = (self._groups.keys())
        
        self._mid = {group:None for group in groups}

        
    def blink(self,color1,color2,eyes=True,chest=True,duration=0.01):
        color1 = [self._colors[color1][rgb] for rgb in ["red","green","blue"]]
        color2 = [self._colors[color2][rgb] for rgb in ["red","green","blue"]]
        _blink(self._leds,
               color1,
               color2,
               eyes=eyes,
               chest=chest,
               duration=duration)
        
    def set(self, color, group, duration=None):

	if isinstance(color, str):
	    color = self._colors[color]

	group = self._groups[group]

	for c in color:
	    for g in group:
		if c == g:
		    for x in group[g]:
                        if duration is None:
			    new_mid = self._leds.post.setIntensity(x, color[c])
                        else :
                            new_mid = self._leds.post.fade(x,color[c],duration)
                        try:
                            self._leds.stop(self._mid[g])
                        except:
                            pass
                        self._mid[g] = new_mid
                        
                    
                
