# -*- coding: utf-8 -*-
#Only for personal debug uses
from setup_environment.environment import Environment
from setup_environment.js_environment import JohnSnowEnvironment
import numpy as np
import matplotlib.pyplot as plt

def draw_grid(gridmap):
  fig = plt.figure(figsize=(18, 9.6))
  ax = fig.add_subplot(111)
  ax.set_title('colorMap')
  plt.imshow(gridmap)
  ax.set_aspect('equal')

  cax = fig.add_axes([0.12, 0.1, 0.78, 0.8])
  cax.get_xaxis().set_visible(False)
  cax.get_yaxis().set_visible(False)
  cax.patch.set_alpha(0)
  cax.set_frame_on(False)
  plt.colorbar(orientation='vertical')
  plt.show()


if __name__ == "__main__":
  env = Environment()
  env.create_gridmap()
  env.setup_walls()
  '''Choose ports setup place here'''
  env.setup_loading_ports_on_row()
  env.setup_unloading_ports_on_bottom()
  draw_grid(env.static_gridmap.array)