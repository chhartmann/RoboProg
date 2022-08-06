print = function(str)
    logWeb(str)
end

Robot = {a1 = 0, a2 = 0, a3 = 0, a4 = 0}

function Robot:new(o, cfg)
    o = o or {}
    setmetatable(o, self)
    self.__index = self
    self.cfg = cfg
    return o
end

function Robot:move()
   setJointAngles(self.a1, self.a2, self.a3, self.a4)
   delay(200)
end

function Robot:moveRel(a1, a2, a3)
    self.a1 = self.a1 + a1
    self.a2 = self.a2 + a2
    self.a3 = self.a3 + a3
    self:move()
end

function Robot:moveHome()
    self.a1 = cfg["home"][1]
    self.a2 = cfg["home"][2]
    self.a3 = cfg["home"][3]
    self.a4 = cfg["home"][4]
    self:move()
end

function Robot:gripperOpen()
   self.a4 = 90
   self:move()
end

function Robot:gripperClose()
   self.a4 = 0
   self:move()
end

print("hello world")
r = Robot:new(getConfig())
r:moveHome()
r:moveRel(0, -30, -30)
r:gripperClose()
r:moveHome()
r:moveRel(-30, 0, 0)
r:gripperOpen()
r:moveHome()
