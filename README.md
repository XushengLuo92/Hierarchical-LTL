we consider the problem of optimally allocating tasks, expressed as a global Linear Temporal Logic (LTL) specification, to teams of mobile robots. We extract these tasks from the Nondeterministic Buchi Automaton (NBA) that models the global LTL and then allocate them to robots by formulating a Mixed Integer Linear Program (MILP) that also respects the temporal constraints between tasks captured in the LTL. We present numerical simulations that demonstrate the correctness and efficiency of the proposed method.
# LTL_MRTA_fullclass


two important technique:
1)  fix the schedule constraints from the vertex to the edge of the same element (line 243-247)
                   # same element, i corresponds to the edge, p corresponds to the vertex
                   
2) treat the starting time of the vertex label as the same as the satisfaction of the previous edge
and the ending time of the vertex label as the same as the satisfaction of the current edge,
no need to consider the vertex label once it is satisfied since the involved robots will not execute any task unless the
edge label is satisfied.