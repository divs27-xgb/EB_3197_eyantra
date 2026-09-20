import os
print([p for p in os.environ["PATH"].split(";") if "EB_3197" in p])