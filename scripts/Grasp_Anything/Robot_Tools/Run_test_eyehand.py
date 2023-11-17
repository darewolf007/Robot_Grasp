import yaml

# 定义包含数组的字典
data = {
    'my_matrix': [
        [1, 2, 3, 4],
        [5, 6, 7, 8],
        [9, 10, 11, 12],
        [13, 14, 15, 16]
    ]
}

# 写入 YAML 文件
with open('output_matrix_data.yaml', 'w') as file:
    yaml.dump(data, file)
