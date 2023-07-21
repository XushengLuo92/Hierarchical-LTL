class Specification():
    def get_task_specification(self, task, case):
        if task == "man":
            return self.get_manipulation_specification(case)
        elif task == "nav":
            return self.get_navigation_specification(case)
        else:
            exit
        
    def get_navigation_specification(self, case):
        hierarchy = []
        if case == 0:
            # ------------------------ task 0 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100_2_1_0 || p200_1_1_0) && <> p300_1_1_0"
            hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> (p2_2_1_0 && <> p3_1_1_0)"
            level_two["p200"] = "<> p4_1_1_0"
            level_two["p300"] = "<> p5_1_1_0"
            # level_two["p300"] = "<> p1_1_1_0"
            hierarchy.append(level_two)
        elif case == 1: 
            # ------------------------ task 1 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100_1_1_0 && <> p1_2_1_0)"
            hierarchy.append(level_one)

            level_two = dict()
            # level_two["p100"] = "<> p2_1_1_0"
            level_two["p100"] = "<> (p2_1_1_0 && <> p4_1_1_0)"
            hierarchy.append(level_two)
        elif case == 2: 
            # ------------------------ task 2 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100_1_1_0 && <> p200_1_1_0)"
            hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> p2_1_1_0 && <> p4_1_1_0"
            level_two["p200"] = "<> (p3_1_1_0 && <> (p5_1_1_0 || p1_1_1_0))"
            hierarchy.append(level_two)
        elif case == 3: 
            # ------------------------ task 3 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100_1_1_0 && <> (p200_1_1_0 && <> p7_1_1_0))"
            hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> (p2_2_1_0 && <> p4_1_1_0)"
            level_two["p200"] = "<> (p3_2_1_0 && <> (p5_1_1_0 || p1_1_1_0)) && <> p6_1_1_0 && !p6_1_1_0 U p3_2_1_0"
            hierarchy.append(level_two)
        elif case == 4: 
            # ------------------------ task 4 -------------------------
            level_one = dict()
            # level_one["p0"] = "<> (p3_2_1_0 && X(p3_2_1_0 U p3_1_1_0))" # && <> p3_1_1_0" # && <> p1_2_1_0"
            level_one["p0"] = "<> (p100_1_1_0 && <> p5_1_1_0))"
            hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> (p200_1_1_0 && <> p300_1_1_0)"
            hierarchy.append(level_two)
            
            level_three = dict()
            # level_three["p200"] = "<> p1_2_1_0 && !p2_2_1_0 U p1_2_1_0"
            level_three["p200"] = "<> (p3_2_1_0 && p3_2_1_0 U p3_1_1_0)"
            level_three['p300'] = "<> p1_2_1_0"
            hierarchy.append(level_three)

        return hierarchy
    
    def get_manipulation_specification(self, case):
        hierarchy = []
        if case == 0:
            # ------------------------ task 0 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100_2_1_0 || p200_1_1_0) && <> p300_1_1_0"
            hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> (p2_2_1_0 && <> p3_1_1_0)"
            level_two["p200"] = "<> p4_1_1_0"
            level_two["p300"] = "<> p5_1_1_0"
            # level_two["p300"] = "<> p1_1_1_0"
            hierarchy.append(level_two)
        elif case == 1: 
            # ------------------------ task 1 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100_1_1_0 && <> p1_2_1_0)"
            hierarchy.append(level_one)

            level_two = dict()
            # level_two["p100"] = "<> p2_1_1_0"
            level_two["p100"] = "<> (p2_1_1_0 && <> p4_1_1_0)"
            hierarchy.append(level_two)
        elif case == 2: 
            # ------------------------ task 2 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100_1_1_0 && <> p200_1_1_0)"
            hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> p2_1_1_0 && <> p4_1_1_0"
            level_two["p200"] = "<> (p3_1_1_0 && <> (p5_1_1_0 || p1_1_1_0))"
            hierarchy.append(level_two)
        elif case == 3: 
            # ------------------------ task 3 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100_1_1_0 && <> (p200_1_1_0 && <> p7_1_1_0))"
            hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> (p2_2_1_0 && <> p4_1_1_0)"
            level_two["p200"] = "<> (p3_2_1_0 && <> (p5_1_1_0 || p1_1_1_0)) && <> p6_1_1_0 && !p6_1_1_0 U p3_2_1_0"
            hierarchy.append(level_two)
        elif case == 4: 
            # ------------------------ task 4 -------------------------
            level_one = dict()
            level_one["p0"] = "<> (p100_1_1_0 && <> p7_1_1_0))"
            hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> (p200_1_1_0 && <> p4_1_1_0)"
            hierarchy.append(level_two)
            
            level_three = dict()
            level_three["p200"] = "<> p3_2_1_0"
            hierarchy.append(level_three)

        return hierarchy