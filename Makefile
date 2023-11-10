module.tar.gz:
	tar czf $@ *.sh .env src/*.py src/pointcloud/*.py requirements.txt 
