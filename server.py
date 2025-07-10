from opcua import ua, Server
server = Server()
server.set_endpoint("opc.tcp://localhost:4839/opc/")
server.set_server_name("MABR")
uri = "http://examples.freeopcua.github.io"
idx = server.register_namespace(uri)
dev = server.nodes.base_object_type.add_object_type(idx, "MyDevice")
server.start()