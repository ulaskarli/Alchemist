from collections import OrderedDict

inits=OrderedDict()
inits["from Lib.ur5.FunctionLibrary import FunctionLib"] = -1
inits["import rospy"] = -1
inits["rospy.init_node(\'gpt\')"] = -1
inits["lib = FunctionLib()"] = -1

def verify_code(file_path): 
    with open(file_path, "r") as file:   
        line_number = 0
        data = file.readlines()
        for line in data:
            for key in inits.keys():
                if key in line and not "#" in line:#key == line[:-1]:
                    inits[key]=line_number
            
            line_number += 1
    
        key_number = 1
        for key in inits.keys():
            val = inits[key]
            if val == -1:
                data.insert(key_number,key+"\n")
                key_number += 1
                inits[key]=key_number
            else: 
                key_number = val+1

    rospy_line_num = inits["rospy.init_node(\'gpt\')"]
    rospy_line = data[rospy_line_num]
    lib_line_num = inits["lib = FunctionLib()"]
    lib_line = data[lib_line_num]

    if lib_line_num < rospy_line_num:
        data[lib_line_num] = rospy_line
        data[rospy_line_num] = lib_line

    with open(file_path, "w") as file:
        file.writelines(data)


verify_code("/home/ulasberkkarli/natural_robot/LLM/gpt_code_3.py")