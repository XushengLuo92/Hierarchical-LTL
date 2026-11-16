"""Render validated original paths or explicitly schematic assembly schedules."""
import argparse
import json
import math
from pathlib import Path
import textwrap
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import imageio_ffmpeg
from matplotlib.animation import FFMpegWriter
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle
from .demo_monitor import Monitor
from .demo_catalog import CARDS, REGIONS

BG, PANEL, INK, MUTED = '#0b1522', '#14263a', '#e9f0f7', '#96acc2'
GOOD, AMBER = '#5cdec4', '#ffc373'
COLORS = ['#60d9ce', '#a4f0df', '#70aaff', '#b0ceff', '#cc98ff', '#e6c9ff']


def draw_map(ax, workspace):
    ax.set(xlim=(-.6,40.6), ylim=(-.5,16.6), aspect='equal', xticks=[], yticks=[])
    ax.set_facecolor(BG)
    for spine in ax.spines.values(): spine.set_visible(False)
    ax.add_collection(PatchCollection([Rectangle((x,y),1,1) for x in range(40) for y in range(15)],
                                      facecolor='#122236',edgecolor='#20344b',linewidth=.35))
    for points in workspace['obstacles'].values():
        ax.add_collection(PatchCollection([Rectangle(p,1,1) for p in points],facecolor='#35485b',edgecolor='#41566c',linewidth=.4))
    for region, points in workspace['regions'].items():
        color = '#70aaff' if region == 'p0' else GOOD if region == 'p7' else '#9bafc8'
        ax.add_collection(PatchCollection([Rectangle((x+.05,y+.05),.9,.9) for x,y in points],facecolor=color,alpha=.24,linewidth=0))
        x,y = np.mean(points,axis=0)+.5
        ax.text(x, 14.4 if region not in ('p0','p7') else y+3.6, REGIONS[region],color=color,fontsize=11,weight='bold',ha='center')


class Presentation:
    def __init__(self,data):
        self.data = data
        self.monitor = Monitor(data)
        self.report = self.monitor.validate()
        if not self.report['valid']:
            raise ValueError(f'Replay validation failed: {self.report}')
        self.horizon = self.monitor.horizon
        self.nav = data['task']=='nav'
        plt.rcParams.update({'font.family':'DejaVu Sans','text.usetex':False})
        self.fig = plt.figure(figsize=(16,9),dpi=120,facecolor=BG)
        self.ax = self.fig.add_axes([.035,.50,.93,.45])
        self.robots = []
        self.bricks = []
        if self.nav:
            draw_map(self.ax,data['workspace'])
            for i,r in enumerate(data['robots']):
                color = COLORS[i]
                line,=self.ax.plot([],[],color=color,lw=2.2,alpha=.5)
                dot=self.ax.scatter([],[],s=180,facecolor=color,edgecolor=BG,linewidth=1.5,zorder=10)
                label=self.ax.text(0,0,str(r['index']+1),ha='center',va='center',color=BG,fontsize=9,weight='bold',zorder=11)
                self.robots.append((r,line,dot,label))
                self.fig.text(.045+i*.155,.967,f'●  Type {r["type"]} · Robot {r["index"]+1}',color=color,fontsize=10)
        else:
            self.draw_assembly()
            self.fig.text(.045,.96,'ASSEMBLY PLAN · SCHEMATIC',color=MUTED,fontsize=10,weight='bold')
            self.fig.text(.89,.96,'● Arm 1',color=COLORS[0],fontsize=10,ha='right')
            self.fig.text(.955,.96,'● Arm 2',color=COLORS[2],fontsize=10,ha='right')
        self.guard=self.fig.text(.045,.475,'',color=AMBER,fontsize=11)
        self.root=self.fig.text(.955,.475,'',color=GOOD,fontsize=11,ha='right',weight='bold')
        self.fig.text(.045,.442,'LTL TASK PROGRESS',color=MUTED,fontsize=9,weight='bold')
        self.fig.text(.955,.442,'F  eventually     U  until     X  next',color=MUTED,fontsize=9,ha='right')
        self.cards=[]
        spec=CARDS[(data['task'],data['case'])]
        cols=3 if len(spec)==6 else 2
        width=(.91-.026*(cols-1))/cols
        for i,(phi,title,formula) in enumerate(spec):
            x=.045+(i%cols)*(width+.026)
            y=.235-(i//cols)*.205
            patch=FancyBboxPatch((x,y),width-.008,.18,boxstyle='round,pad=.008,rounding_size=.009',
                                transform=self.fig.transFigure,facecolor=PANEL,edgecolor='#2b4056',linewidth=1.2)
            self.fig.add_artist(patch)
            self.fig.text(x+.008,y+.137,title,color=INK,fontsize=12,weight='bold')
            status=self.fig.text(x+width-.022,y+.14,'',color=MUTED,fontsize=9,ha='right')
            self.fig.text(x+.008,y+.10,formula,color=MUTED,fontsize=8.8)
            desc=self.fig.text(x+.008,y+.058,'',color=INK,fontsize=10,va='top')
            bar=Rectangle((x+.008,y+.012),0,.004,transform=self.fig.transFigure,facecolor=GOOD,edgecolor='none')
            self.fig.add_artist(bar)
            leaves=self.descendants(phi)
            stages=[s for s in data['stages'] if s['subtask'][0] in leaves]
            self.cards.append((phi,patch,status,desc,bar,width,stages))
        self.update(0)

    def descendants(self,phi):
        return {phi}.union(*(self.descendants(c) for c in self.monitor.children[phi]))

    def draw_assembly(self):
        ax=self.ax
        ax.set(xlim=(0,25),ylim=(0,9),aspect='equal',xticks=[],yticks=[])
        ax.set_facecolor(BG)
        for s in ax.spines.values(): s.set_visible(False)
        # Logical placement diagrams, intentionally not kinematic arm simulations.
        layouts={}
        if self.data['case']==5:
            for ids,x,letter in [([3,4,5],2,'I'),([6,7,8],8,'C'),([9,10,11],14,'L'),([1,2],20,'!')]:
                ax.text(x+1.2,8.3,letter,color=MUTED,ha='center',fontsize=20,weight='bold')
                for j,brick in enumerate(ids): layouts[brick]=(x,1.2+j*1.9,2.4,1.1)
        else:
            layers=[[1,2,3,4],[5,6,9,10],[7,8,11,12],[13,14],[15,16,17],[18]]
            for layer,ids in enumerate(layers):
                width=2.5
                left=12.5-len(ids)*(width+.25)/2
                for j,brick in enumerate(ids): layouts[brick]=(left+j*(width+.25),.55+layer*1.28,width,.85)
                ax.text(3.5,.95+layer*1.28,f'Layer {layer+1}',ha='right',color=MUTED,fontsize=10)
        for brick,(x,y,w,h) in layouts.items():
            region=f'p{brick}'
            stage=next(s for s in self.data['stages'] if region in s['essential_robot_edge'])
            robot=stage['essential_robot_edge'][region][0]
            color=COLORS[robot[1]*2]
            patch=FancyBboxPatch((x,y),w,h,boxstyle='round,pad=.04,rounding_size=.1',facecolor=PANEL,edgecolor='#40556b',lw=1.3)
            ax.add_patch(patch)
            label=ax.text(x+w/2,y+h/2,f'p{brick}',color=MUTED,ha='center',va='center',fontsize=11,weight='bold')
            self.bricks.append((stage['time_element'],robot,color,patch,label,brick))
        ax.text(24.7,.15,'Placement order and arm assignment',ha='right',color=MUTED,fontsize=9)

    def update(self,step):
        step=min(step,self.horizon)
        t=int(step)
        if self.nav:
            for r,line,dot,label in self.robots:
                path=np.asarray(r['path'])+.5
                a=min(t,len(path)-1); b=min(a+1,len(path)-1)
                p=path[a]*(1-(step-t))+path[b]*(step-t)
                trail=np.vstack([path[max(0,a-10):a+1],p])
                line.set_data(trail[:,0],trail[:,1]);dot.set_offsets([p]);label.set_position(p)
        else:
            for when,robot,color,patch,label,brick in self.bricks:
                done=step>=when
                patch.set_facecolor(color if done else PANEL)
                patch.set_edgecolor(color if done else '#40556b')
                label.set_color(BG if done else MUTED)
                label.set_text(f'p{brick} · A{robot[1]+1}' if done else f'p{brick}')
        done=sum(s['time_element']<=step for s in self.data['stages'])
        self.root.set_text('p0 · mission complete' if self.monitor.status('p0',step)=='done' else f'p0 · {done}/{len(self.data["stages"])} milestones')
        guard=''
        if self.nav and self.data['case']==5:
            release=self.monitor.completed['p201']; health=self.monitor.completed['p204']
            arrival=next(s['time_element'] for s in self.data['stages'] if s['subtask'][0]=='p204')
            guard=('Furniture handoff complete · Type 1 may leave' if step>=release else
                   'Type 1 must stay at furniture until Type 3 arrives')
            if step<arrival: guard+='   |   Grocery restricted for Type 2'
        elif self.nav and self.data['case']==6: guard='Shopping stages: furniture + outdoor  →  health + grocery  →  electronics + pet  →  deliver'
        elif self.nav: guard='Outdoor collection: Type 2 OR Type 3 · the planner chooses one branch'
        elif self.data['case']==5: guard='Complete I, C, and L before starting !'
        else: guard='Layer 1  →  Layer 2  →  Layer 3  →  Layer 4  →  Layer 5  →  Layer 6'
        self.guard.set_text(guard)
        for phi,patch,status,desc,bar,width,stages in self.cards:
            state=self.monitor.status(phi,step)
            color=GOOD if state=='done' else AMBER if state=='active' else MUTED
            status.set_text('NOT SELECTED' if state=='unused' else state.upper())
            status.set_color(color);patch.set_edgecolor(color if state in ('done','active') else '#2b4056')
            count=sum(s['time_element']<=step for s in stages)
            bar.set_width((width-.032)*count/max(1,len(stages)))
            future=next((s for s in stages if s['time_element']>step),None)
            if state=='unused': text='Alternative branch not needed'
            elif future:
                tasks=[]
                for region,robots in future['essential_robot_edge'].items():
                    for kind,index in robots:
                        name=f'T{kind} · R{index+1}' if self.nav else f'Arm {index+1}'
                        tasks.append(f'{name} → {REGIONS[region] if self.nav else region}')
                text=' / '.join(tasks)
            else: text=f'All {len(stages)} milestones complete'
            desc.set_text(text)
        return []

    def render(self,path,speed=2,fps=30):
        plt.rcParams['animation.ffmpeg_path']=imageio_ffmpeg.get_ffmpeg_exe()
        frames=math.ceil(self.horizon/speed*fps)+fps*2+1
        writer=FFMpegWriter(fps=fps,codec='libx264',extra_args=['-crf','20','-pix_fmt','yuv420p','-movflags','+faststart'])
        with writer.saving(self.fig,str(path),dpi=120):
            for frame in range(frames):
                self.update(frame/fps*speed)
                writer.grab_frame(facecolor=BG)
        self.update(min(self.horizon*.35,self.horizon))
        self.fig.savefig(Path(path).with_suffix('.png'),facecolor=BG,dpi=120)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('replay',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--still',type=float)
    args=parser.parse_args()
    data=json.loads(args.replay.read_text())
    show=Presentation(data)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.still is not None:
        show.update(args.still);show.fig.savefig(args.output,facecolor=BG,dpi=120)
    else:
        args.output.with_suffix('.validation.json').write_text(json.dumps(show.report,indent=2)+'\n')
        args.output.with_suffix('.json').write_text(json.dumps(data,indent=2)+'\n')
        show.render(args.output,speed=2 if data['task']=='nav' else .7)
