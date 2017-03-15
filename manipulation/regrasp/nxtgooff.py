import robotcon.nextage as nxtcon

if __name__=='__main__':
    nxts = nxtcon.NxtSocket()
    nxts.initialize()
    nxts.gooff()