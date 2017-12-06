import cmd_node
pub=None
##
## @brief      Class for kubs.
##
class kubs:
	##
	## @brief      Constructs the object.
	##
	## @param      self     The object
	## @param      kubs_id  The kubs identifier
	##
	def __init__(self, kubs_id):
		self.kubs_id = kubs_id
		self.pos = state.homePos[self.kubs_id]
		self.velocity = None
		self.is_team_yellow = False

		self.reset()
	##
	## @brief      { function_description }
	##
	## @param      self  The object
	##
	## @return     { description_of_the_return_value }
	##
	def reset(self):
		self.dribbler = False
		self.vx = 0.0
		self.vy = 0.0
		self.vw = 0.0
		self.power = 0.0
	##
	## @brief      { function_description }
	##
	## @param      self          The object
	## @param      target_point  The target point
	## @param      state         The state
	##
	## @return     { description_of_the_return_value }
	##
	def move(self,vx,vy):
		self.vx = vx
		self.vy = vy
		pass
	##
	## @brief      { function_description }
	##
	## @param      self  The object
	##
	## @return     { description_of_the_return_value }
	##
	def dribble(self,dribbler):
		self.dribbler = dribbler
		pass
	##
	## @brief      { function_description }
	##
	## @param      self   The object
	## @param      angle  The angle
	##
	## @return     { description_of_the_return_value }
	##

	def turn(self, vw):
		self.vw = vw
		pass
	##
	## @brief      { function_description }
	##
	## @param      self   The object
	## @param      power  The power
	##
	## @return     { description_of_the_return_value }
	##
	def kick(self,power):
		self.power = power
		pass

	##
	## @brief      { function_description }
	##
	## @param      self   The object
	## @param      state  The state
	##
	## @return     { description_of_the_return_value }
	##
	def execute(self,state):
		cmd_node.send_command(pub, state.isteamyellow, self.kubs_id, self.vx, self.vy, self.vw, self.power, self.dribbler)	
		self.reset()



	##
	## @brief      Gets the position.
	##
	## @param      self   The object
	## @param      state  The state
	##
	## @return     The position.
	##
	def get_pos(self,state):
		self.pos = state.homePos[self.kubs_id]
		return self.pos


	
		





		

