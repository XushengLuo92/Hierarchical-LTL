"""Render all solved examples, map, GIF previews, and validation reports."""
from pathlib import Path
import json
import os
import subprocess
import sys
import imageio_ffmpeg

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from hierarchical_ltl.presentation import Presentation, draw_map, BG
import matplotlib.pyplot as plt

if __name__=='__main__':
    output=ROOT/'demo/generated'
    output.mkdir(parents=True,exist_ok=True)
    for scenario in range(1,6):
        name=f'scenario_{scenario:02d}'
        data=json.loads((ROOT/'data/presentation'/f'{name}.json').read_text())
        show=Presentation(data)
        (output/f'{name}.json').write_text(json.dumps(data,indent=2)+'\n')
        (output/f'{name}.validation.json').write_text(json.dumps(show.report,indent=2)+'\n')
        print(f'Rendering {name}: {show.horizon} steps',flush=True)
        show.render(output/f'{name}.mp4',speed=2 if data['task']=='nav' else .7)
        plt.close(show.fig)
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-v','error','-i',str(output/f'{name}.mp4'),'-filter_complex',
                        'fps=8,scale=800:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse',
                        '-loop','0',str(output/f'{name}.gif')],check=True)
    data=json.loads((output/'scenario_01.json').read_text())
    fig,ax=plt.subplots(figsize=(16,6.6),facecolor=BG)
    draw_map(ax,data['workspace'])
    fig.savefig(output/'environment.png',facecolor=BG,dpi=120,bbox_inches='tight')
    print('All five demos and workspace map saved.',flush=True)
