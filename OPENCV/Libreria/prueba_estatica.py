from lineAnalyzer import lineAnalyzer
import cv2

#Inicializamos un objeto
analizador = lineAnalyzer(1, debug = ['line'])
analizador.begin()

#Analizamos una foto
cosa = analizador.processPicture('3.jpg')
print type(analizador.frame_debug_line)
resultado = cv2.resize(analizador.frame_debug_line, (1280,720),interpolation = cv2.INTER_AREA)
# Display the resulting frame
cv2.imshow('frame',resultado)
cv2.waitKey(0)

# When everything done, release the capture
analizador.camera.release()
cv2.destroyAllWindows()
