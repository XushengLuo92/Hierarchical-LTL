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
            level_one["p0"] = "<> (p100_1_1_0 && <> p5_1_1_1))"
            hierarchy.append(level_one)

            level_two = dict()
            level_two["p100"] = "<> (p200_1_1_0 && <> p300_1_1_0)"
            hierarchy.append(level_two)
            
            level_three = dict()
            # level_three["p200"] = "<> p1_2_1_0 && !p2_2_1_0 U p1_2_1_0"
            level_three["p200"] = "<> (p3_2_1_1 && X (p3_2_1_1 U p3_1_1_1))"
            level_three['p300'] = "<> p1_2_1_1"
            hierarchy.append(level_three)
        elif case == 5:
            level_one = dict()
            level_one["p0"] = '<> p101_1_1_0 && <> p102_1_1_0'
            hierarchy.append(level_one)
            level_two = dict()
            level_two['p101'] = '<> (p6_3_1_3 && <> (p201_1_1_0 && <> (p202_1_1_0 && <> p203_1_1_0)))'
            level_two['p102'] = '<> (p204_1_1_0 &&  <> p205_1_1_0)'
            hierarchy.append(level_two)
            level_three = dict()
            # p0 dock
            # p1 grocery p2 health p3 outdors p4 pet p5 furniture p6 electronics 
            # p7 packing area
            level_three['p201'] = '<> (p5_1_1_1 && X (p5_1_1_1 U p5_3_1_3))'
            level_three['p202'] = '<> p3_1_1_1 && <> p4_1_1_1 && <> (p7_3_1_3 &&  <> p0_3_1_3)'
            level_three['p203'] = '<> (p7_1_1_1 && <> p0_1_1_1)'
            level_three['p204'] = '<> p2_2_1_2 && <> p1_2_1_2 && !p1_2_1_2 U p2_2_1_2'
            level_three['p205'] = '<> (p7_2_1_2 && <> p0_2_1_2)'
            hierarchy.append(level_three)
            # level_four = dict()
            # level_four['p301'] = '<> (p5_1_1_1 && X (p5_1_1_1 U p5_3_1_3))'
            # hierarchy.append(level_four)
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