tec.up:
#	@xhost +
	@docker start tec
tec.down:
#	@xhost +
	@docker stop tec
tec.restart:
#	@xhost +
	@docker restart tec
tec.shell:
#	@xhost +	
	@docker exec -it tec bash
tec.build:
	@docker build -t tec ./
# tec.intelcreate:
# 	@./runROS2Intel.bash