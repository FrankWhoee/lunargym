import gym
from gym.spaces import discrete
from gym.spaces import space
import random
import math
import numpy as np
from mapline import mapline


class lunarEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    reward_range = (-float('inf'), float('inf'))
    spec = None

    def __init__(self):
        self.action_set = [(-1, 0), (-1, 1), (0, 0), (0, 1), (1, 0), (1, 1)]
        self.action_space = discrete.Discrete(6)
        self.observation_space = space.Space(shape=(318,), dtype=np.float32)
        self.map = []
        self.done = False
        self.info = {}

        self.map_width = 1500
        self.map_height = 750
        self.piece_width = 20

        self.isThrusting = False

        self.angle = 0
        self.fuel = 500
        self.velX = 0
        self.velY = 0
        self.reward = 0

        self.lander_w = 19
        self.lander_h = 18

        self.x = self.map_width / 2
        self.y = self.map_height / 2

        # Constants
        self.thrust = 0.05
        self.gravity = -0.0162
        self.viewer = None
        self.generateNewMap()
        if self.getPieceUnderLander().y > self.y or self.getPieceUnderLander().yo > self.y:
            self.y = max(self.getPieceUnderLander().y, self.getPieceUnderLander().yo) + 50

    def get_observations(self):
        float_map = [ml.get_coordinates() for ml in self.map]
        observations = []
        for line in float_map:
            for float in line:
                observations.append(float)
        observations.extend([self.map_width, self.map_height, self.piece_width, self.angle, self.fuel, self.velX,
                             self.velY, self.reward, self.lander_h, self.lander_w, self.x, self.y, self.thrust,
                             self.gravity])
        return observations

    def getPieceUnderLander(self):
        return self.map[int(self.x / self.piece_width)]

    def respawn(self):
        self.angle = 0
        self.velX = 0
        self.velY = 0
        self.x = self.map_width / 2
        self.y = self.map_height / 2
        if self.getPieceUnderLander().y > self.y or self.getPieceUnderLander().yo > self.y:
            self.y = max(self.getPieceUnderLander().y, self.getPieceUnderLander().yo) + 50

    def landingArea(self):
        originI = self.map.index(self.getPieceUnderLander())
        rightPieces = 0
        for i in range(originI + 1, len(self.map)):
            if not self.map[i].isFlat():
                break
            rightPieces += 1

        leftPieces = 0
        for i in range(originI - 1, 0, -1):
            if not self.map[i].isFlat():
                break
            leftPieces += 1
        return leftPieces + rightPieces + 1

    def check(self):
        if self.isColliding():
            area = self.landingArea()
            bonus = 1.5 / (0.5 * float(area))

            if abs(self.velX) < 0.1 and abs(self.velY) < 0.1 and self.getPieceUnderLander().isFlat():
                addedScore = int(float(250) * bonus)
                self.reward += addedScore
                self.respawn()
            elif abs(self.velX) < 0.2 and abs(self.velY) < 0.2 and self.getPieceUnderLander().isFlat():
                addedScore = int(float(150) * bonus)
                self.reward += addedScore
                self.respawn()
            elif abs(self.velX) < 0.3 and abs(self.velY) < 0.3 and self.getPieceUnderLander().isFlat():
                addedScore = int(float(50) * bonus)
                self.reward += addedScore
                self.respawn()
            elif self.getPieceUnderLander().isFlat():
                addedScore = int(float(5) * bonus)
                self.reward += addedScore
                self.respawn()
            else:
                self.reward -= min(250,self.reward)
                self.reset()
            # print(self.reward)

        if self.fuel <= 0 or self.y < 0 or self.y > self.map_height:
            self.done = True
            self.reset()

    def step(self, action_index):
        action = self.action_set[action_index]
        dtheta = action[0]
        thrust = action[1]
        self.check()

        if self.done:
            return [self.get_observations(), self.reward, self.done, self.info]
        if dtheta == 1:
            self.angle += 2
        elif dtheta == -1:
            self.angle -= 2
        if thrust == 1 and self.fuel > 0:
            # print("Thrusting...")
            self.velX += self.thrust * math.sin(math.radians(self.angle))
            self.velY += self.thrust * math.cos(math.radians(self.angle))
            if abs(self.velX) < 0.001:
                self.velX = 0
            if abs(self.velY) < 0.001:
                self.velY = 0
            self.fuel -= 1
        # else:
            # print("Not thrusting...")
        self.velY += self.gravity

        self.x += self.velX
        self.y += self.velY
        return [self.get_observations(), self.reward, self.done, self.info]

    def getLanderBorders(self):
        borders = []
        left = mapline(self.x - self.lander_w / 2, self.x - self.lander_w / 2, self.y + self.lander_h / 2,
                       self.y - self.lander_h / 2)
        right = mapline(self.x + self.lander_w / 2, self.x + self.lander_w / 2, self.y + self.lander_h / 2,
                        self.y - self.lander_h / 2)
        top = mapline(self.x - self.lander_w / 2, self.x + self.lander_w / 2, self.y + self.lander_h / 2,
                      self.y + self.lander_h / 2)
        bottom = mapline(self.x - self.lander_w / 2, self.x + self.lander_w / 2, self.y - self.lander_h / 2,
                         self.y - self.lander_h / 2)

        borders.append(left)
        borders.append(right)
        borders.append(top)
        borders.append(bottom)
        return borders

    def isColliding(self):
        landerborders = self.getLanderBorders()
        for mapline in self.map:
            for border in landerborders:
                if border.isVertical or mapline.isVertical:
                    x_inter = border.x if border.isVertical else mapline.x
                    y_inter = mapline.m * x_inter + mapline.b if border.isVertical else border.m * x_inter + border.b
                else:
                    try:
                        x_inter = (mapline.b - border.b) / (border.m - mapline.m)
                    except:
                        x_inter = float('inf')
                    y_inter = border.m * x_inter + border.b

                interLander = max(border.x, border.xo) >= x_inter >= min(border.x,
                                                                         border.xo) and max(
                    border.y, border.yo) >= y_inter >= min(border.y, border.yo)
                interTerrain = max(mapline.x, mapline.xo) >= x_inter >= min(mapline.x,
                                                                            mapline.xo) and max(
                    mapline.y, mapline.yo) >= y_inter >= min(mapline.y, mapline.yo)

                if interLander and interTerrain:
                    return True
        return False

    def reset(self):
        self.map = []

        self.done = False
        self.reward = 0
        self.info = {}

        self.angle = 0
        self.fuel = 500
        self.velX = 0
        self.velY = 0
        self.reward = 0

        self.x = self.map_width / 2
        self.y = self.map_height / 2
        if self.viewer is not None:
            self.viewer.close()
        else:
            self.viewer = None
        self.generateNewMap()
        self.viewer = None
        if self.getPieceUnderLander().y > self.y or self.getPieceUnderLander().yo > self.y:
            self.y = max(self.getPieceUnderLander().y, self.getPieceUnderLander().yo) + 50
        return self.get_observations()

    def render(self, mode='human'):
        screen_width = self.map_width
        screen_height = self.map_height
        from gym.envs.classic_control import rendering

        if self.viewer is None:
            self.viewer = rendering.Viewer(screen_width, screen_height)
            l, r, t, b = -self.lander_w / 2, self.lander_w / 2, self.lander_h / 2, -self.lander_h / 2
            lander = rendering.Image(fname="lander.png", width=self.lander_w, height=self.lander_h)
            self.landertrans = rendering.Transform()
            lander.add_attr(self.landertrans)
            self.viewer.add_geom(lander)

            up = rendering.FilledPolygon([(1320,820),(1300,820),(1300, 800), (1320,800)])
            self.up_c = rendering.Color([0,0,0,1])
            up.add_attr(self.up_c)
            self.viewer.add_geom(up)

            for ml in self.map:
                line = rendering.Line(start=(ml.xo, ml.yo), end=(ml.x, ml.y))
                self.linetrans = rendering.Transform()
                line.add_attr(self.linetrans)
                self.viewer.add_geom(line)
            self.newMap = False

        if self.isThrusting:
            self.up_c.enable()
        else:
            self.up_c.disable()

        self.landertrans.set_translation(self.x, self.y)
        self.landertrans.set_rotation(math.radians(self.angle))

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def generateNewMap(self):
        for i in range(0, self.map_width + 1, self.piece_width):
            xo = i
            x = i + self.piece_width
            yo = 0 if len(self.map) == 0 else self.map[((i / self.piece_width) - 1)].y
            y = max((yo + random.uniform(-40, 60)), 0)
            if random.randint(0, 100) > 50:
                y = yo
            newline = mapline(xo, x, yo, y)
            self.map.append(newline)


