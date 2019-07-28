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
        self.reward = 0
        self.info = {}

        self.map_width = 1500
        self.map_height = 750
        self.piece_width = 20;

        self.angle = 0
        self.fuel = 500
        self.velX = 0
        self.velY = 0
        self.score = 0

        self.lander_w = 19
        self.lander_h = 18

        self.x = self.map_width / 2
        self.y = self.map_height / 2

        # Constants
        self.thrust = 0.005
        self.gravity = 0.00162

        self.generateNewMap()

    def get_observations(self):
        float_map = [ml.get_coordinates() for ml in self.map]
        observations = []
        for line in float_map:
            for float in line:
                observations.append(float)
        observations.extend([self.map_width, self.map_height, self.piece_width, self.angle, self.fuel, self.velX,
                             self.velY, self.score, self.lander_h, self.lander_w, self.x, self.y, self.thrust,
                             self.gravity])
        return observations

    def getPieceUnderLander(self):
        return self.map.get(int(self.x / self.piece_width))

    def respawn(self):
        self.angle = 0
        self.fuel = 500
        self.velX = 0
        self.velY = 0

    def landingArea(self):
        originI = self.getPieceUnderLander()
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
                self.score += addedScore
            elif abs(self.velX) < 0.2 and abs(self.velY) < 0.2 and self.getPieceUnderLander().isFlat():
                addedScore = int(float(150) * bonus)
                self.score += addedScore
            elif abs(self.velX) < 0.3 and abs(self.velY) < 0.3 and self.getPieceUnderLander().isFlat():
                addedScore = int(float(50) * bonus)
                self.score += addedScore
            else:
                self.score -= 250
            self.respawn()

        if self.fuel == 0:
            self.done == True

    def step(self, action_index):
        action = self.action_set[action_index]
        dtheta = action[0]
        thrust = action[1]
        self.check()
        self.reward == self.score
        if self.done:
            return [self.map, self.reward, self.done]
        if dtheta == 1:
            self.angle += 2
        elif dtheta == -1:
            self.angle -= 2

        if thrust == 1 and self.fuel > 0:
            self.velX += self.thrust * math.sin(math.radians(self.angle))
            self.velY += self.thrust * math.cos(math.radians(self.angle))
            if abs(self.velX) < 0.001:
                self.velX = 0
            if abs(self.velY) < 0.001:
                self.velY = 0
            self.fuel -= 0.1
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
                        x_inter = math.inf
                    y_inter = border.m * x_inter + border.b

                interLander = max(border.x, border.xo) >= x_inter >= min(border.x,
                                                                         border.xo) and max(
                    border.y, border.yo) >= y_inter >= min(border.y, border.yo)
                interTerrain = max(mapline.x, mapline.xo) >= x_inter >= min(mapline.x,
                                                                            mapline.xo) and max(
                    mapline.y, mapline.yo) >= y_inter >= min(mapline.y, mapline.yo)

                return interLander and interTerrain
        return False

    def reset(self):
        self.map = []

        self.map_width = 1500
        self.map_height = 750
        self.piece_width = 20;

        self.angle = 0
        self.fuel = 500
        self.velX = 0
        self.velY = 0
        self.score = 0

        self.lander_w = 19
        self.lander_h = 18

        self.x = self.map_width / 2
        self.y = self.map_height / 2

        # Constants
        self.thrust = 0.005 + random.uniform(-0.001, 0.001)
        self.gravity = 0.00162 + random.uniform(-0.001, 0.001)
        self.generateNewMap()
        return self.get_observations()

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    def generateNewMap(self):
        for i in range(0, self.map_width + 1, self.piece_width):
            xo = i
            x = i + self.piece_width
            yo = self.map_height if len(self.map) == 0 else self.map[((i / self.piece_width) - 1)].y
            y = min((yo + random.uniform(0, 100) - 55), self.map_height)
            if random.randint(0, 100) > 0.75:
                y = yo
            newline = mapline(xo, x, yo, y)
            self.map.append(newline)
