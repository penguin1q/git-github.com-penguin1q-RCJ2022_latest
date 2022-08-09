from Camera_Module import Camera_Module
import time
import cv2

def main():
    camera = Camera_Module()
    pretime = time.perf_counter()

    #画面を写す場合は waitKey 移さないなら Trueでもok   
    while cv2.waitKey(1) < 0:
        dt = time.perf_counter() - pretime
        pretime = time.perf_counter()
        print(f"{dt:.3} {1/dt:.3}")
        #@param 
        #(orange_freq, blue_freq, yellow_freq, freq, raw=False, dposx = 0, dposy = 0)
        #オレンジ、ブルー、イエローの測定タイミング[3, 8]３回目と８回目これはめんどい許して
        #[]の場合、上を除く残りすべて読む. freqは母数(0にリセットするタイミング)
        #
        #raw＝返り値の変更 
        #
        #dposx, dposy ミラーの位置がずれているときなどソフト側の微調整
        #
        #@return
        #list(オレンジ), list(ブルー), list(イエロー)
        #raw -> False
        #座標データとオブジェクトの幅と高さ(オレンジのみ1つ)
        #raw -> True
        #左上の座標とオブジェクトの幅と高さ
        orange, blue, yellow = camera.read_pos_data([], [3, 8], [2, 7], 10)
        
        #print(orange, blue, yellow)

if __name__ == "__main__":
	main() 