import time
from subprocess import Popen
from runtime_naive_search import runtime_search

if __name__ == "__main__":

    image_slides_proc = Popen("python playFreestyleVideo.py")
    time.sleep(1)

    runtime_search()