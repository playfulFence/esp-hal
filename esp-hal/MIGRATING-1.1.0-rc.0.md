# Migration Guide from 1.0.0 to 1.1.0-rc.0

## I2S one-shot DMA uses buffers

I2S one-shot DMA APIs now use the DMA buffer types/traits instead of raw slices.

```diff
 let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
 let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
-i2s_tx.write_dma(&tx_buffer)?;
-i2s_rx.read_dma(&mut rx_buffer)?;
+let tx_transfer = i2s_tx.write_dma(dma_tx_buf).expect("start tx dma");
+let (_i2s_tx, _dma_tx_buf) = tx_transfer.wait().expect("finish tx dma");
+let rx_transfer = i2s_rx
+    .read_dma(dma_rx_buf.len(), dma_rx_buf)
+    .expect("start rx dma");
+let (_i2s_rx, _dma_rx_buf) = rx_transfer.wait().expect("finish rx dma");
```

- `write_dma` / `read_dma` now use `DmaTxBuffer` / `DmaRxBuffer` inputs and
  return owning transfer handles.
- `read_dma` now takes an explicit byte count.
- async one-shot APIs now take `&mut DmaTxBuf` / `&mut DmaRxBuf`.
