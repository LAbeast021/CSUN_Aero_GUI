def saveLoggs (inforArrays , eventArrays):
    sorted_array = sorted(eventArrays, key=lambda x: x[1], reverse=True)
    print(sorted_array)
    print("====================================================================================================")
    print(inforArrays)