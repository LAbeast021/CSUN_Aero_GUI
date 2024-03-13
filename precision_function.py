# Application Name: experimenting
# Programmer: Evan Thomas
# Last Updated Date:October 9th, 2022
# Origin Date: October 9th, 2022
# Purpose: file was created to allow me to experiment with certain functions.


# This function was created to add zeroes after the decimal point for 
# waypoint file formatting.
def precision(variable,precision):
    chars = "."
    has = all(char in str(variable) for char in chars)
    if (has == False):
        variable = str(variable)+"."
    x=str(variable).split(".")
    output=(x[0]+"."+(x[1]).ljust(precision,"0"))
    return output
# print("testing defenition:\n")
# num = 11
# print(precision(num,8))



