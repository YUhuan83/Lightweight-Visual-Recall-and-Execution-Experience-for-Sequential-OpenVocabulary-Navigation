"""Ordered mission hold; controller commands cannot override an uncertain goal."""
import json
import math
import time


class ExecutionHoldReceiver:
    def __init__(self):
        self.held = False
        self.owner = None
        self.revision = -1
        self.wall_time = float('-inf')

    def receive(self, message):
        try:
            data = json.loads(message.data)
            owner,revision,held,wall = (data[k] for k in ('owner','revision','held','wall_time'))
            if (not isinstance(owner,str) or not owner or type(revision) is not int or revision < 0
                    or type(held) is not bool or type(wall) not in (float,int)
                    or not math.isfinite(wall) or not 0 <= time.time()-wall <= 1.
                    or (owner == self.owner and revision <= self.revision)
                    or (owner != self.owner and wall <= self.wall_time)):
                return False
            self.owner,self.revision,self.wall_time,self.held = owner,revision,wall,held
            return True
        except (ValueError,KeyError,TypeError):
            return False
