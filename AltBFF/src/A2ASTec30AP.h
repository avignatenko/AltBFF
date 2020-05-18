
class Sim;
class Model;
namespace bffcl { struct CLInput; }

class A2AStec30AP
{
public:

	A2AStec30AP(Sim& sim, Model& model) : sim_(sim), model_(model)
	{

	}
	
	void process(bffcl::CLInput& input);

private:

	Sim& sim_;
	Model& model_;

};