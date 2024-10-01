import cv2
from matplotlib import pyplot as plt

window_name = "origin"
png_path = "/home/lab-station2/Desktop/mp-release-fa24/src/mp1/src/test.png"
image = cv2.imread(png_path)
cv2.imshow(window_name,image)
cv2.waitKey(0)
cv2.destroyAllWindows()
plt.figure(1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.show()