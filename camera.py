import cscore
import numpy

def main():
    cscore.CameraServer.enableLogging()
    camera = cscore.CameraServer.startAutomaticCapture()

    camera.setResolution(640, 480)

    sink = cscore.CameraServer.getVideo()
    outputStream = cscore.CameraServer.putVideo("ClimbCamera", 640, 480)

    mat = numpy.zeros(shape=(480, 640, 3), dtype=numpy.uint8)

    while True:
        time, mat = sink.grabFrame(mat)
        if time == 0:
            outputStream.notifyError(sink.getError())
            continue

        # Flip the camera feed right-side-up
        numpy.flipud(mat)
        
        # Give the output stream a new image to display
        outputStream.putFrame(mat)
