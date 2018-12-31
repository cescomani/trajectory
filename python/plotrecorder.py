"""
 A simple Python module for recording matplotlib animation
 This tool use convert command of ImageMagick
"""

import matplotlib.pyplot as plt
from subprocess import call, Popen, PIPE      #use to execute shell command in python script 

iframe = 0
makeRecord = True

def save_frame(folder):
    """
    Save a frame for movie
    """
    if makeRecord:
       global iframe
       plt.savefig(folder + "/" + "image" + '{0:04d}'.format(iframe) + '.png')
       iframe += 1


def save_movie(fname, d_pause):
    """
    Save movie as gif
    """
    if makeRecord:
       cmd = "convert -delay " + str(int(d_pause * 100)) + " recoder*.png " + fname
       call(cmd, shell=True)
       cmd = "rm recoder*.png"
       call(cmd, shell=True)

def save_movie_as_mp4(folder):
    """
    Save 
    """	
    if makeRecord:
       cmd = "rm -f demo.mp4 && ffmpeg -i image%04d.png -c:v libx264 -pix_fmt yuv420p demo.mp4 && rm image*.png"
       P=Popen(cmd, cwd=folder, stderr=PIPE, shell=True)
       P.wait()
       P.terminate()
       
      	 	
