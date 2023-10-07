# Hierarchical LTL
Past research into robotic planning with temporal logic specifications, notably Linear Temporal Logic (LTL), was largely based on singular formulas for individual or groups of robots. But with increasing task complexity, LTL formulas unavoidably grow lengthy, complicating interpretation and specification generation, and straining the computational capacities of the planners. In order to maximize the potential of LTL specifications, we capitalized on the intrinsic structure of tasks and introduced a hierarchical structure to LTL specifications. In contrast to the "flat" structure, our hierarchical model has multiple levels of compositional specifications and offers benefits such as greater syntactic brevity, improved interpretability, and more efficient planning. To address tasks under this hierarchical temporal logic structure, we formulated a decomposition-based method. Each specification is first broken down into a range of temporally interrelated  sub-tasks. We further mine the temporal relations among the sub-tasks of different specifications within the hierarchy. Subsequently, a Mixed Integer Linear Program is utilized to generate a spatio-temporal plan for each robot. Our hierarchical LTL specifications were experimentally applied to domains of robotic navigation and manipulation. Results from extensive simulation studies illustrated both the enhanced expressive potential of the hierarchical form and the efficacy of the proposed method.

# Example
Demo video can be accessible via this [link](https://youtu.be/YbmYmq1RmhI?si=eI1JoR5qIEuFKwWi).
## Case 1
```bash
python main.py --task=nav --case=5 --vis --dot
```
# Case 2
```bash
python main.py --task=nav --case=6 --vis --dot
```
# Case 3
```bash
python main.py --task=nav --case=8 --vis --dot
```
# Citation
```
@article{luo2023robotic,
  title={Robotic Planning under Hierarchical Temporal Logic Specifications},
  author={Luo, Xusheng and Xu, Shaojun and Liu, Ruixuan and Liu, Changliu},
  journal={arXiv preprint arXiv:2308.10393},
  year={2023}
}
```