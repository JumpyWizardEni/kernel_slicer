import os
import imageio

FILES_NUM = 500

#TODO почему-то не работала сортировка с разным числом файлов, надо доделать
def create_gif(dir):
    images = []
    l = [i for i in range(0, FILES_NUM)]
    for file_name in l:
        file_path = os.path.join(dir, f"{file_name}.jpeg")
        images.append(imageio.imread(file_path))
    imageio.mimsave("images/res.gif", images, duration=1.0/30)

if __name__ == '__main__':
    create_gif("images/")
